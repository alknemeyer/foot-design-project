from sympy import Matrix as Mat

from pyomo.environ import (
    ConcreteModel, RangeSet, Set, Param, Var, Constraint, Objective, value as pyovalue  # type: ignore
)

from typing import List, Tuple

import physical_education as pe


class VariableList:
    """Access using VAR_LIST[fe,cp] with fe = 1..nfe, cp = 1..ncp"""

    def __init__(self, m) -> None:
        self._var_list = [
            [self.prepare_var_list(m, fe, cp) for cp in m.cp] for fe in m.fe
        ]

    def prepare_var_list(self, m, fe, cp) -> List:
        #Fp = m.Fp[fe, cp, :] if model_type.startswith('max') is True else []
        return [*m.q[fe, cp, :], *m.dq[fe, cp, :], *m.ddq[fe, cp, :],
                *m.Tc[fe, :], *m.Fr[fe, cp, :],
                *m.GRFx[fe, cp, :], m.GRFy[fe, cp]]

    def __getitem__(self, idx: Tuple):
        """`idx` is a tuple of (finite_element, collocation_point)"""
        assert (1 <= idx[0] <= len(self._var_list)
                and 1 <= idx[1] <= len(self._var_list[0]))

        return self._var_list[idx[0]-1][idx[1]-1]


def pyomo_model(EOM_func, foot_height_func, foot_dx_func,
                nfe: int, total_time: float,
                collocation: str = 'radau',
                timestep_bounds: Tuple[float, float] = (0.8, 1.2)):
    from numpy import pi as π
    assert collocation in ('euler', 'radau')

    ncp: int = 3 if collocation == 'radau' else 1

    m = ConcreteModel('Closed kinematic hopper')
    m.fe = RangeSet(nfe)
    m.cp = RangeSet(ncp)
    m.hm0 = Param(initialize=total_time/nfe)
    m.hm = Var(m.fe, bounds=timestep_bounds)

    m.vars = Set(initialize=('xb', 'yb', 'thb',
                             'thul', 'thur',
                             'thll', 'thlr'), ordered=True)

    m.q = Var(m.fe, m.cp, m.vars, bounds=(-10, 10))
    m.dq = Var(m.fe, m.cp, m.vars)
    m.ddq = Var(m.fe, m.cp, m.vars)

    # for fe,cp in utils.get_indexes_(m, one_based=True):
    #    m.q[fe,cp,'thur'].setlb( 0.01)
    #    m.q[fe,cp,'thul'].setub(-0.01)

    m.Tc_set = Set(initialize=('l', 'r'), ordered=True)
    m.Tc = Var(m.fe, m.Tc_set, bounds=(-2, 2))

    # geometry constraint forces
    m.Fr_set = Set(initialize=('N_x', 'N_y'))
    m.Fr = Var(m.fe, m.cp, m.Fr_set, bounds=(-5, 5))

    # ground reaction forces
    m.posneg_set = Set(initialize=('+', '-'), ordered=True)
    m.GRFx = Var(m.fe, m.cp, m.posneg_set, bounds=(0, 10))
    m.GRFy = Var(m.fe, m.cp, bounds=(0, 10))

    variables = VariableList(m)

    # foot related stuff ----------------------------------------
    m.mu = Param(initialize=1.0, name='friction coefficient')

    m.foot_dx = Var(m.fe, m.cp, m.posneg_set, bounds=(0, None))
    m.foot_height = Var(m.fe, m.cp, bounds=(0, None))
    #m.gamma = Var(m.fe, m.cp, bounds=(0, None), name='foot xy velocity magnitude')

    # utils.def_dummy_var(m, 'foot_height', (m.fe, m.cp),
    #                  lambda m,fe,cp: foot_height_func(*variables[fe,cp])  if not (fe==1 and cp<ncp) else Constraint.Skip)

    @m.Constraint(m.fe, m.cp)
    def foot_height_dummy(m, fe, cp):
        return (m.foot_height[fe, cp] == foot_height_func(variables[fe, cp])
                if not (fe == 1 and cp < ncp) else Constraint.Skip)

    @m.Constraint(m.fe, m.cp)
    def foot_dx_dummy(m, fe, cp):
        return (m.foot_dx[fe, cp, '+'] - m.foot_dx[fe, cp, '-'] == foot_dx_func(variables[fe, cp])
                if not (fe == 1 and cp < ncp) else Constraint.Skip)

    # @m.Constraint(m.fe, m.cp, m.posneg_set)
    # def gamma_dummy(m, fe, cp, pn):
    #     return m.gamma[fe,cp] >= (m.foot_dx[fe,cp,pn] if pn == '+' else -m.foot_dx[fe,cp,pn])

    @m.Constraint(m.fe, m.cp)
    def friction_cone(m, fe, cp):
        return m.mu * m.GRFy[fe, cp] >= sum(m.GRFx[fe, cp, :]) if not (fe == 1 and cp < ncp) else Constraint.Skip

    m.contact_penalty = Var(m.fe,            bounds=(0, 100))
    m.friction_penalty = Var(m.fe,           bounds=(0, 100))
    m.slip_penalty = Var(m.fe, m.posneg_set, bounds=(0, 100))

    @m.Constraint(m.fe)  # z[i+1]*GRFz[i] ≈ 0
    def contact_complementarity(m, fe):  # foot on ground OR GRF
        if fe < m.fe[-1]:
            α = sum(m.foot_height[fe+1, :])
            β = sum(m.GRFy[fe, :])
            return α * β <= m.contact_penalty[fe]
        else:
            return Constraint.Skip

    @m.Constraint(m.fe)  # (μ * GRFy - Σ GRFx) * γ ≈ 0
    # on friction cone OR no horizontal velocity
    def friction_complementarity(m, fe):
        if fe < m.fe[-1]:
            α = (m.mu * sum(m.GRFy[fe, :]) - sum(m.GRFx[fe, :, :]))
            β = sum(m.foot_dx[fe, :, :])
            return α * β <= m.friction_penalty[fe]
        else:
            return Constraint.Skip

    @m.Constraint(m.fe, m.posneg_set)  # GRFx * (γ + dxᵀ*Dᵢ) ≈ 0
    # horizontal force to right OR movement to right
    def slip_complementarity(m, fe, pn):
        if fe < m.fe[-1]:
            α = sum(m.GRFx[fe, :, pn])
            β = sum(m.foot_dx[fe, :, pn])
            return α * β <= m.slip_penalty[fe, pn]
        else:
            return Constraint.Skip
    
    for fe in m.fe:
        m.contact_penalty[fe].value = 0
        m.friction_penalty[fe].value = 0
        for p in m.posneg_set:
            m.slip_penalty[fe,p].value = 0

    # next, add in equations of motion (this covers the dynamics and geometry constraints)
    @m.Constraint(m.fe, m.cp, range(len(EOM_func)))
    def EOM(m, fe, cp, i):
        return (EOM_func[i](variables[fe, cp]) == 0
                if not (fe == 1 and cp < ncp) else Constraint.Skip)

    from physical_education.collocation import implicit_euler, radau_3
    if collocation == 'euler':
        m.interp_q = Constraint(
            m.fe, m.cp, m.vars, rule=implicit_euler(m.q,  m.dq))
        m.interp_dq = Constraint(
            m.fe, m.cp, m.vars, rule=implicit_euler(m.dq, m.ddq))
    else:
        m.interp_q = Constraint(
            m.fe, m.cp, m.vars, rule=radau_3(m.q,  m.dq))
        m.interp_dq = Constraint(
            m.fe, m.cp, m.vars, rule=radau_3(m.dq, m.ddq))

    # relative angle constraint: can't cross legs
    @m.Constraint(m.fe, m.cp)
    def rel_angle_constraint(m, fe, cp):
        return m.q[fe, cp, 'thur'] >= m.q[fe, cp, 'thul']

    return m


def make_plots(m):
    import matplotlib.pyplot as plt

    def default_plot(plt, title='', yax=''):
        plt.title(title)
        plt.ylabel(yax)
        plt.gcf().set_size_inches(10, 6)
        plt.grid(True)
        plt.show()

    # positions and angles
    q = pe.utils.get_vals(m.q, (m.vars,))
    plt.plot(q[:, 0, :])
    plt.legend(list(m.vars))
    default_plot(plt, '$q$')

    # velocities
    dq = pe.utils.get_vals(m.dq, (m.vars,))
    plt.plot(dq[:, 0, :])
    plt.legend(list(m.vars))
    default_plot(plt, '$\\dot{q}$')

    # input torques
    Tc = pe.utils.get_vals(m.Tc, (m.Tc_set,))
    plt.step(range(m.fe[-1]), Tc)
    default_plot(plt, '$T_c$', '$Nm$\n(normalized to BW)')

    # ground reaction forces
    Lx = pe.utils.get_vals(m.GRFx, (m.posneg_set,))
    Ly = pe.utils.get_vals(m.GRFy, tuple())
    foot_height = pe.utils.get_vals(m.foot_height, tuple())

    plt.plot(foot_height[:, 0])
    plt.plot(Lx[:, 0])
    plt.plot(Lx[:, 0, 0] - Lx[:, 0, 1])
    plt.legend(['foot height', 'GRFx+', 'GRFx-', 'GRF'])
    default_plot(plt, '$GRF_x$', '$N$\n(normalized to BW)')

    plt.plot(foot_height[:, 0])
    plt.plot(Ly[:, 0])
    plt.legend(['foot height', 'GRF_y'])
    default_plot(plt, '$GRF_y$', '$N$\n(normalized to BW)')

    # connection forces
    Fr = pe.utils.get_vals(m.Fr, (m.Fr_set,))
    plt.plot(Fr[:, 0, :])
    default_plot(plt, '$F_r$', '$N$\n(normalized to BW)')


# MAKE A VID
def get_joint_data(m, joint_position_funcs):
    """vars_in_EOM = [*q, *dq, *ddq, *τ, *Fr, *Lx, Ly]"""
    q = pe.utils.get_vals(m.q, (m.vars,))
    dq = pe.utils.get_vals(m.dq, (m.vars,))
    ddq = pe.utils.get_vals(m.ddq, (m.vars,))
    Tc = pe.utils.get_vals(m.Tc, (m.Tc_set,))
    Fr = pe.utils.get_vals(m.Fr, (m.Fr_set,))
    Lx = pe.utils.get_vals(m.GRFx, (m.posneg_set,))
    Ly = pe.utils.get_vals(m.GRFy, tuple())

    nfe = m.fe[-1]
    cp = 0
    vs = [[*q[fe, cp, :], *dq[fe, cp, :], *ddq[fe, cp, :],
           *Tc[fe, :], *Fr[fe, cp, :], *Lx[fe, cp, :], Ly[fe, cp]]
          for fe in range(nfe)]

    out = {}
    for key in joint_position_funcs.keys():
        fx, fy = joint_position_funcs[key]
        out[key] = [(fx(v), fy(v)) for v in vs]

    return out


def move_rectangle(r, xy, θ, w, h):
    """r = rectangle, xy = center, θ = angle in rads, w = width, h = height"""
    from numpy import sin, cos
    x = xy[0] - w/2*cos(θ) + h/2*sin(θ)
    y = xy[1] - w/2*cos(θ) - h/2*sin(θ)
    r.set_xy([x, y])
    r.angle = θ


def make_animation(m, joint_position_funcs, h_m: float, body_width: float,
                   title: str = 'closed kinematic chain hopper'):
    import matplotlib.pyplot as plt
    import matplotlib.animation
    import sys
    plt.rcParams['animation.ffmpeg_path'] = 'ffmpeg'  # type: ignore

    # some plotting admin
    fig = plt.figure(figsize=(10, 6), dpi=60)
    ax = plt.axes(xlim=(-0.5, 3), ylim=(0, 2))
    plt.title(title, fontdict={'fontsize': 18})
    ax.set_aspect('equal')  # type: ignore
    # ax.axis('off')

    # the ground
    ground = ax.plot([-5, 5], [0, 0], color='green',  # type: ignore
                     linewidth=2)[0]

    # Define the different elements in the animation
    body = plt.Rectangle([0, 0], body_width, body_width,  # type: ignore
                         color='red', angle=0)
    ax.add_patch(body)  # type: ignore
    rods = [ax.plot([], [], color='grey', linewidth=2)[0]  # type: ignore
            for i in range(4)]

    data = get_joint_data(m, joint_position_funcs)

    def animate(i):
        xb, yb = data['body'][i]
        ax.scatter([xb], [yb], color='gray', s=1)  # type: ignore
        body.set_xy([xb - body_width/2, yb - body_width/2])
        thb = m.q[i+1, 1, 'thb'].value
        thb = 0 if thb is None else thb  # * 180/3.14
        #move_rectangle(body, [xb, yb], θ=thb, w=rb, h=rb)

        xls, yls = data['l-shou'][i]
        xlk, ylk = data['l-knee'][i]
        xlf, ylf = data['l-foot'][i]
        rods[0].set_data([xls, xlk], [yls, ylk])
        rods[1].set_data([xlk, xlf], [ylk, ylf])

        xrs, yrs = data['r-shou'][i]
        xrk, yrk = data['r-knee'][i]
        xrf, yrf = data['r-foot'][i]
        rods[2].set_data([xrs, xrk], [yrs, yrk])
        rods[3].set_data([xrk, xrf], [yrk, yrf])
        #ax.scatter([xrf], [yrf], color='green', s=1)

    nfe = m.fe[-1]
    anim = matplotlib.animation.FuncAnimation(fig, animate, frames=nfe,
                                              interval=1000*h_m,
                                              repeat_delay=1000)

    if 'ipykernel' in sys.modules:  # or 'IPython'?
        from IPython.core.display import display, HTML
        plt.close(anim._fig)
        display(HTML(anim.to_html5_video()))
    else:
        plt.show(anim._fig)

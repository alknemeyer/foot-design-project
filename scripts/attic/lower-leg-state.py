# def gen_lower_leg_angle_funcs():
#     import sympy as sp
#     th_ul, th_ur, th_ll, th_lr, d = sp.symbols('th_ul th_ur th_ll thlr d', real=True)
#     l1s, l2s, dth_ul, dth_ur = sp.symbols('l1 l2 dth_ul dth_ur', real=True)

#     a = th_ul - th_ur
#     b = th_ur + (180 - th_lr)
#     c = th_ll + (180 - th_ul)
#     d = 360 - (a + b + c)

#     th_llf, th_lrf = sp.solve([
#         # 360 - (a + b + c) - d,
#         b - c,
#         sp.Eq(
#             2*l1s**2 * (1 - sp.cos(a)),
#             2*l2s**2 * (1 - sp.cos(d))
#         )
#     ], (th_ll, th_lr))[0]

#     vec = lambda *args: sp.Matrix(args)
#     dth_llf, dth_lrf = sp.simplify(
#         vec(th_llf, th_lrf).jacobian([th_ul, th_ur]) @ vec(dth_ul, dth_ur)
#     )

#     print("""
#     def lower_leg_angles(th_ul: float, th_ur: float, dth_ul: float, dth_ur: float):
#         import math
#         return {
#             'th_ll': %s,
#             'th_lr': %s,
#             'dth_ll': %s,
#             'dth_lr': %s,
#         }
#     """ % tuple(map(sp.pycode, (th_llf, th_lrf, dth_llf, dth_lrf))))

# def lower_leg_angles(th_ul: float, th_ur: float, dth_ul: float, dth_ur: float):
#     l1 = lib.l1
#     l2 = lib.l2
#     import math
#     return {
#         'th_ll': (1/2)*th_ul + (1/2)*th_ur + (1/2)*math.acos((l1**2*math.cos(th_ul - th_ur) - l1**2 + l2**2)/l2**2),
#         'th_lr': (1/2)*th_ul + (1/2)*th_ur - 1/2*math.acos((l1**2*math.cos(th_ul - th_ur) - l1**2 + l2**2)/l2**2),
#         'dth_ll': (1/2)*dth_ul*l1**2*math.sin(th_ul - th_ur)/(l2**2*math.sqrt(1 - (l1**2*math.cos(th_ul - th_ur) - l1**2 + l2**2)**2/l2**4)) + (1/2)*dth_ul - 1/2*dth_ur*l1**2*math.sin(th_ul - th_ur)/(l2**2*math.sqrt(1 - (l1**2*math.cos(th_ul - th_ur) - l1**2 + l2**2)**2/l2**4)) + (1/2)*dth_ur,
#         'dth_lr': -1/2*dth_ul*l1**2*math.sin(th_ul - th_ur)/(l2**2*math.sqrt(1 - (l1**2*math.cos(th_ul - th_ur) - l1**2 + l2**2)**2/l2**4)) + (1/2)*dth_ul + (1/2)*dth_ur*l1**2*math.sin(th_ul - th_ur)/(l2**2*math.sqrt(1 - (l1**2*math.cos(th_ul - th_ur) - l1**2 + l2**2)**2/l2**4)) + (1/2)*dth_ur,
#     }

# def read_leg_state(odrv: 'odrive.ODrive'):
#     upper_leg_state = upper_leg_angles(odrv)
#     lower_leg_state = lower_leg_angles(**upper_leg_state)
#     return {**upper_leg_state, **lower_leg_state}

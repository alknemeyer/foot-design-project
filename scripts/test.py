import time
from . import lib
from .lib import PositionControl
from typing import Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from odrive import ODrive


def increment(odrv: 'ODrive', nloops: int = 3, delta: float = 100):
    """delta > 0 to push off ground, delta < 0 to go lower"""
    with PositionControl(odrv):
        lib.fast_gains(odrv)
        for _ in range(nloops):
            odrv.axis0.controller.pos_setpoint -= delta
            odrv.axis1.controller.pos_setpoint += delta
            time.sleep(1)
            print('.'*50)


def slipping(odrv: 'ODrive'):
    def pos(ax): return ax.encoder.pos_estimate

    ip0, ip1 = pos(odrv.axis0), pos(odrv.axis1)
    idiff = ip0 - ip1

    while True:
        try:
            p0, p1 = pos(odrv.axis0), pos(odrv.axis1)
            print(f'{p0-ip0:7.3f}, {p1-ip1:7.3f}, {p0-p1-idiff:7.3f}')
            time.sleep(1)
        except KeyboardInterrupt:
            break


def controllooptimes(nloops: int = 10_000, odrv: Optional['ODrive'] = None):
    from .controller_generated import impedance_control, foot_state_polar

    # random initial float
    th_ul = th_ur = dth_ul = dth_ur = kp = kd = 0.1
    r, th, dr, dth = foot_state_polar(th_ul, th_ur, dth_ul, dth_ur)

    ddr_controller = lib.PDControl(r, dr, kp, kd)
    ddth_controller = lib.PDControl(th, dth, kp, kd)
    looptimes = []
    tstart = time.time()

    for _ in range(nloops):
        if odrv is not None:
            th_ul, th_ur, dth_ul, dth_ur = lib.upper_leg_angles(odrv)

        r, th, dr, dth = foot_state_polar(th_ul, th_ur, dth_ul, dth_ur)

        # inputs: ddr_setpoint, ddth_setpoint
        curr0, curr1 = impedance_control(
            ddr_setpoint=ddr_controller(r, dr),
            ddth_setpoint=ddth_controller(th, dth),
            th_ul=th_ul, th_ur=th_ur, dth_ul=dth_ul, dth_ur=dth_ur,
            L_x=0, L_y=0, Kt=lib.torque_constant,
        )

        if odrv is not None:
            lib.set_current(odrv, motor0=curr0, motor1=curr1)

        looptimes.append(time.time())

    runtime = time.time() - tstart
    print('average loop time:', 1000*runtime/nloops, 'ms')

    import matplotlib.pyplot as plt
    import numpy as np
    plt.style.use('seaborn')
    plt.plot(np.diff(np.array(looptimes)) * 1000)
    plt.title(f'loop times in milliseconds. HIL: {odrv is not None}')
    plt.show()


def slow_position_control(odrv: 'ODrive', vel_limit: float = 180):
    from functools import partial
    setangle = partial(lib.set_to_nearest_angle, odrv, vel_limit=vel_limit)

    with PositionControl(odrv):
        lib.fast_gains(odrv)
        for x, y in ((0, -0.4), (0.2, -0.2), (-0.2, -0.2), (0, -0.2)):
            th0, th1 = lib.foot_xy_to_th_deg(x_m=x, y_m=y)
            print(f'changing to: {th0=} {th1=} {x=} {y=}')
            setangle(th0, motornum=0)
            setangle(th1, motornum=1)
            time.sleep(4)

import time
from . import lib
from .lib import PositionControl
from typing import Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from odrive import ODrive


def increment(odrv: 'ODrive', nloops: int = 3, delta: float = 100):
    """
    Increment the angle of each motor `nloops` times, each time
    by `delta`, so that they move a small amount in opposite
    directions (ie either closer or further apart)
    """
    with PositionControl(odrv):
        lib.fast_gains(odrv)
        for _ in range(nloops):
            odrv.axis0.controller.pos_setpoint -= delta
            odrv.axis1.controller.pos_setpoint += delta
            print('Incremented angle')
            time.sleep(1)


def slipping(odrv: 'ODrive'):
    """
    Test for slipping in the encoder. Setup: straighten the
    leg as much as possible (or otherwise fix it so that the
    relative angle between the motors is constant) and then
    rotate the leg a full circle

    Each position should meausure the correct encoder count
    (4000 at the time of writing) and the difference should
    stay "relatively close" to zero. A bit of drift is fine
    (at the time of writing, it can be around +- 20, likely
    due to some wiggle/loose coupling/stretch etc)
    """
    def pos(ax):
        return ax.encoder.pos_estimate

    # initial encoder position
    initial_p0, initial_p1 = pos(odrv.axis0), pos(odrv.axis1)

    print('angle 0, angle 1, difference')
    while True:
        try:
            p0 = pos(odrv.axis0) - initial_p0
            p1 = pos(odrv.axis1) - initial_p1
            print(f'{p0:7.3f}, {p1:7.3f}, {p0-p1:7.3f}')
            time.sleep(1)
        except KeyboardInterrupt:
            break


def controllooptimes(nloops: int = 10_000, odrv: Optional['ODrive'] = None):
    """
    Estimate control loop times
    """
    from .controller_generated import impedance_control, foot_state_polar
    import numpy as np

    # random initial values
    th_ul, th_ur, dth_ul, dth_ur, kp, kd = (.1, .2, .3, .4, .5, .6)

    r, th, dr, dth = foot_state_polar(
        th_ul, th_ur, dth_ul, dth_ur, lib.l3x, lib.l3y)

    ddr_controller = lib.PDControl(r, dr, kp, kd)
    ddth_controller = lib.PDControl(th, dth, kp, kd)
    looptimes = []
    tstart = time.time()

    for _ in range(nloops):
        if odrv is not None:
            th_ul, th_ur, dth_ul, dth_ur = lib.upper_leg_angles(odrv)

        r, th, dr, dth = foot_state_polar(
            th_ul, th_ur, dth_ul, dth_ur, lib.l3x, lib.l3y)

        # inputs: ddr_setpoint, ddth_setpoint
        curr0, curr1 = impedance_control(
            ddr_setpoint=ddr_controller(r, dr),
            ddth_setpoint=ddth_controller(th, dth),
            th_ul=th_ul, th_ur=th_ur, dth_ul=dth_ul, dth_ur=dth_ur,
            L_x=0, L_y=0, l3x=lib.l3x, l3y=lib.l3y, Kt=lib.torque_constant,
        )

        if odrv is not None:
            lib.set_current(odrv, motor0=curr0, motor1=curr1)

        looptimes.append(time.time())

    runtime = time.time() - tstart
    print(f'average loop time: {1000*runtime/nloops:.3f} ms')

    import matplotlib.pyplot as plt
    plt.style.use('seaborn')
    plt.plot(np.diff(np.array(looptimes)) * 1000)
    plt.title(f'loop times in milliseconds. Using ODrive: {odrv is not None}')
    plt.show()


def slow_position_control(odrv: 'ODrive', vel_limit: float = 180):
    """
    Check that a slow position control works by moving the foot
    to a few positions while under a velocity limit (see Trajectory
    Control in the ODrive docs)
    """
    from functools import partial
    setangle = partial(lib.set_to_nearest_angle, odrv, vel_limit=vel_limit)

    with PositionControl(odrv):
        lib.fast_gains(odrv)
        for x, y in ((0, -lib.max_length), (0.2, -0.2), (-0.2, -0.2), (0, -0.2)):
            th0, th1 = lib.foot_xy_to_th_deg(x_m=x, y_m=y)
            print(f'changing to: {th0=} {th1=} {x=} {y=}')
            setangle(th0, motornum=0)
            setangle(th1, motornum=1)
            time.sleep(4)

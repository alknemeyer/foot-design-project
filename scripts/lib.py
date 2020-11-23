"""
ax0 = left motor (when looking from front)
    = right/front leg (same view)
    decreasing setpoint (neg) -> clockwise

ax1 = right motor (when looking from front)
    = left/back leg (same view)
    increasing setpoint (pos) -> clockwise
"""
import time
from odrive.enums import *
from odrive.utils import dump_errors
from typing import Literal, Optional, Tuple, TYPE_CHECKING
import math

import logging  # for optoforce. Nothing else logs
logging.basicConfig(
    format='%(asctime)s | %(levelname)s | %(name)s: %(message)s',
    datefmt='%I:%M:%S',
    level=logging.INFO,
)

if TYPE_CHECKING:
    from odrive import ODrive

l1: float = 0.1375
l2: float = 0.250
l3: float = 0.030

mu: float = 0.090  # roughly correct
ml: float = 0.090  # short
# m_otherone: float = 0.096  # long
mb: float = 4.460 - 2*mu - 2*ml

encoder_cpr: int = 4*500
gear_ratio: int = 2


class PositionControl:
    def __init__(self, odrv: 'ODrive'):
        self.odrv = odrv
        self.axes = (self.odrv.axis0, self.odrv.axis1)

    def __enter__(self):
        for ax in self.axes:
            ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            ax.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL

    def __exit__(self, *_):
        for ax in self.axes:
            ax.requested_state = AXIS_STATE_IDLE

        dump_errors(self.odrv, clear=True)


# https://github.com/madcowswe/ODrive/blob/fw-v0.4.12/docs/getting-started.md#current-control
class CurrentControl:
    def __init__(self, odrv: 'ODrive'):
        self.odrv = odrv
        self.axes = (self.odrv.axis0, self.odrv.axis1)

    def __enter__(self):
        for ax in self.axes:
            ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            # ax.controller.config.control_mode = CTRL_MODE_TORQUE_CONTROL
            ax.controller.config.control_mode = CTRL_MODE_CURRENT_CONTROL  # type: ignore

    def __exit__(self, *_):
        for ax in self.axes:
            ax.requested_state = AXIS_STATE_IDLE

        dump_errors(self.odrv, clear=True)


#=#=#=#=#=#=#=#=# CONVERSION #=#=#=#=#=#=#=#=#


def foot_xy_to_th_deg(x_m: float, y_m: float) -> Tuple[float, float]:
    """
    returns `(theta_1, theta_2)` in degrees given desired foot
    position `x`, `y`, relative to the axle.
    The angles are 0 when aligned along positive x-axis,
    and increase when going CCW

    ## Examples:
    >>> foot_xy_to_th_deg(0, -0.200)  # foot slightly below center
    (20.758625854720844, -191.6713670789533)

    >>> foot_xy_to_th_deg(0, -0.300)  # foot straight down
    (-21.701601471794238, -152.5640828112919)

    >>> foot_xy_to_th_deg(0.250, -0.270)  # foot forward
    (-5.804085049611694, -85.3146096067127)
    """
    r = math.sqrt(x_m**2 + y_m**2)
    phi = math.atan2(y_m, x_m)

    th1 = phi + math.acos(
        ((l2+l3)**2 - r**2 - l1**2) / (-2*r*l1)
    )

    # p = prime
    xp = (x_m - l1*math.cos(th1)) * l2/(l2+l3) + l1*math.cos(th1)
    yp = (y_m - l1*math.sin(th1)) * l2/(l2+l3) + l1*math.sin(th1)
    rp = math.sqrt(xp**2 + yp**2)
    phip = math.atan2(yp, xp)

    lamb = math.acos((l2**2 - rp**2 - l1**2)/(-2*rp*l1))
    th2 = phip - lamb

    return (math.degrees(th1), math.degrees(th2))


def test_foot_xy_to_th_deg():
    import matplotlib.pyplot as plt
    from math import sin, cos, radians as r
    """
    from scripts.lib import foot_xy_to_th_deg, l1, l2, l3
    """
    plt.cla()
    for x, y in [(0.0, -0.3), (0.3, -0.1)]:
        th1, th2 = foot_xy_to_th_deg(x, y)
        plt.scatter([x], [y])
        for th in (th1, th2):
            _x, _y = (cos(r(th))*l1, sin(r(th))*l1)
            plt.plot([0, _x, x], [0, _y, y])
        plt.grid(True)
        plt.show()


def encoder_count_to_motor_angle_deg(count: int) -> float:
    return count / encoder_cpr * 360


def encoder_count_to_leg_angle_deg(count: int) -> float:
    return encoder_count_to_motor_angle_deg(count) / gear_ratio


def motor_angle_deg_to_encoder_count(angle_deg: float) -> int:
    """
    >>> motor_angle_deg_to_encoder_count(0)
    0
    >>> motor_angle_deg_to_encoder_count(90) == encoder_cpr/4
    """
    return int(angle_deg * encoder_cpr / 360)


def leg_angle_deg_to_encoder_count(angle_deg: float) -> float:
    return gear_ratio * motor_angle_deg_to_encoder_count(angle_deg)


# If eg. two of the motor wires get swapped for one of the motors,
# the direction of rotation would change (unless ODrive accounts
# for that?). You could account for this in code by eg. changing
# one of the functions below to return +x
def l_dir(x: float): return -x
def r_dir(x: float): return -x


def upper_leg_angles(odrv: 'ODrive'):
    """
    Test:
    >>> for i in range(10):
    ...     print(upper_leg_angles(odrv0))
    ...     time.sleep(1)
    """
    e0 = odrv.axis0.encoder
    e1 = odrv.axis1.encoder
    c2a = encoder_count_to_leg_angle_deg
    th_ul = c2a(int(e1.pos_estimate))
    th_ur = c2a(int(e0.pos_estimate))
    dth_ul = c2a(int(e1.vel_estimate))
    dth_ur = c2a(int(e0.vel_estimate))
    return th_ul, th_ur, dth_ul, dth_ur
    # return {
    #     'th_ul': c2a(int(e1.pos_estimate)),
    #     'th_ur': c2a(int(e0.pos_estimate)),
    #     'dth_ul': c2a(int(e1.vel_estimate)),
    #     'dth_ur': c2a(int(e0.vel_estimate)),
    # }


#=#=#=#=#=#=#=#=# GAINS #=#=#=#=#=#=#=#=#

def set_gains(odrv0: 'ODrive',
              vel_scale: float = 1,
              pos_scale: float = 1,
              bandwidth_hz: float = 10.
              ):
    for ax in (odrv0.axis0, odrv0.axis1):
        conf = ax.controller.config

        # [A/(counts/s)]
        conf.vel_gain = 8.0 / 10000.0 * vel_scale

        # [(counts/s) / counts]
        conf.pos_gain = 80.0 * pos_scale

        # [A/((counts/s) * s)]
        conf.vel_integrator_gain = 0.5 * bandwidth_hz * conf.vel_gain


def slow_gains(odrv0: 'ODrive'):
    set_gains(odrv0, vel_scale=0.5, pos_scale=0.5, bandwidth_hz=5.)


def medium_gains(odrv0: 'ODrive'):
    set_gains(odrv0, vel_scale=0.8, pos_scale=0.8, bandwidth_hz=8.)


def fast_gains(odrv0: 'ODrive'):
    set_gains(odrv0)


def nearest_cpr(curr: float, dest: float) -> float:
    """
    >>> nearest_cpr(1200, 3000)
    3000.0

    >>> nearest_cpr(1700, 4000)
    0.0

    >>> nearest_cpr(1200, -3000)
    1000.0

    >>> for curr in range(-10_000, 10_000, 15):
    ...     for dest in range(-10_000, 10_000, 17):
    ...         nearest_cpr(curr, dest)
    """
    lim = encoder_cpr * gear_ratio
    error = dest - curr + lim/2
    n = error//lim
    assert -lim/2 <= (dest - n*lim) - curr <= lim/2, \
        f"Couldn't get nearest cpr! args: {curr=} {dest=}"
    return dest - n*lim


#=#=#=#=#=#=#=#=# ZEROING MOTORS #=#=#=#=#=#=#=#=#

# first approximation -- do it using index
th0, th1 = foot_xy_to_th_deg(x_m=0.417, y_m=0)
almost_zero_deg = (-853.765625, -1489.765625)  # TODO: update!
ZERO_DEG_POS = (
    almost_zero_deg[0] - r_dir(leg_angle_deg_to_encoder_count(th0)),
    almost_zero_deg[1] - l_dir(leg_angle_deg_to_encoder_count(th1)),
)
del th0, th1, almost_zero_deg


def zero_motors_no_index_straightdown(odrv0: 'ODrive'):
    almost_straight_down = (
        odrv0.axis0.encoder.pos_estimate,
        odrv0.axis1.encoder.pos_estimate,
    )
    # zero deg = right
    th0, th1 = foot_xy_to_th_deg(x_m=0, y_m=-0.417)
    global ZERO_DEG_POS
    ZERO_DEG_POS = (
        almost_straight_down[0] - r_dir(leg_angle_deg_to_encoder_count(th0)),
        almost_straight_down[1] - l_dir(leg_angle_deg_to_encoder_count(th1)),
    )

    # above is a slightly more accurate version of:
    # ZERO_DEG_POS = (
    #     almost_straight_down[0] - encoder_cpr//2,
    #     almost_straight_down[1] - encoder_cpr//2,
    # )


#=#=#=#=#=#=#=#=# ACTUALLY MOVING TO POSITIONS #=#=#=#=#=#=#=#=#

def set_to_nearest_angle(odrv0: 'ODrive',
                         th: float,
                         motornum: int,
                         vel_limit: Optional[float] = None):
    ax = odrv0.axis0 if motornum == 0 else odrv0.axis1

    # ax.controller.pos_setpoint would be more repeatable?
    curr = ax.encoder.pos_estimate
    dest = ZERO_DEG_POS[motornum] + r_dir(leg_angle_deg_to_encoder_count(th))
    dest_nearest = nearest_cpr(curr=curr, dest=dest)

    # print(f'{th=} {curr=} {dest=} {dest_nearest=}')
    if vel_limit is None:
        ax.controller.pos_setpoint = dest_nearest
    else:
        # https://github.com/madcowswe/ODrive/blob/fw-v0.4.12/docs/getting-started.md#trajectory-control
        deg2count = leg_angle_deg_to_encoder_count
        ax.trap_traj.config.vel_limit = deg2count(vel_limit)
        ax.trap_traj.config.accel_limit = deg2count(vel_limit/3)
        ax.trap_traj.config.decel_limit = deg2count(vel_limit/3)
        ax.controller.move_to_pos(dest_nearest)


def set_foot_position(odrv0: 'ODrive', x_m: float, y_m: float,
                      gains: Literal['slow', 'medium', 'fast', ''] = '',
                      vel_limit: Optional[float] = None,
                      sleep: float = 0.):
    assert odrv0 is not None
    if gains == 'slow':
        slow_gains(odrv0)
    elif gains == 'medium':
        medium_gains(odrv0)
    elif gains == 'fast':
        fast_gains(odrv0)

    th0, th1 = foot_xy_to_th_deg(x_m=x_m, y_m=y_m)
    set_to_nearest_angle(odrv0, th0, motornum=0, vel_limit=vel_limit)
    set_to_nearest_angle(odrv0, th1, motornum=1, vel_limit=vel_limit)

    if sleep > 0:
        time.sleep(sleep)


def set_current(odrv: 'ODrive', motor0: float, motor1: float):
    odrv.axis0.controller.current_setpoint = motor0
    odrv.axis1.controller.current_setpoint = motor1


from dataclasses import dataclass
@dataclass
class PDControl:
    x0: float
    dx0: float
    kp: float
    kd: float

    def __call__(self, x: float, dx: float) -> float:
        kp = self.kp
        kd = self.kd
        x0 = self.x0
        dx0 = self.dx0
        return kp * (x0 - x) + kd * (dx0 - dx)

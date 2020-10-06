"""
    $ ipython
    >>> %load_ext autoreload
    >>> %autoreload 2
    >>> from scripts import lib
"""
import odrive
from odrive.enums import *
from odrive.utils import dump_errors
from typing import Tuple, TYPE_CHECKING
import math

if TYPE_CHECKING:
    from odrive import ODrive

l1: float = 0.1375
l2: float = 0.250
l3: float = 0.030
encoder_cpr: int = 4*500
gear_ratio: int = 2


class PositionControl:
    def __init__(self, odrv: 'ODrive'):
        self.odrv = odrv

    def __enter__(self):
        for ax in [self.odrv.axis0, self.odrv.axis1]:
            # ax.controller.pos_setpoint = ax.encoder.pos_estimate
            ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            ax.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL

    def __exit__(self, *_):
        for ax in [self.odrv.axis0, self.odrv.axis1]:
            ax.requested_state = AXIS_STATE_IDLE

        dump_errors(self.odrv, clear=True)


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


def current_control(L_x: float, L_y: float,
                    th_ul: float, th_ll: float,
                    th_lr: float, th_ur: float,
                    dth_ll: float, dth_lr: float):
    "TODO: still need to get correct masses and lengths!"
    import math
    current_input = (
        2.5*L_x*math.cos(th_ul) + 2.5*L_y*math.sin(th_ul) + 0.075*dth_ll**2*math.sin(th_ll - th_ul) -
        0.0375*dth_lr**2*math.sin(th_lr - th_ur) - 7.3575 *
        math.sin(th_ul) + 3.67875*math.sin(th_ur),
        -1.25*L_x*math.cos(th_ul) - 1.25*L_y*math.sin(th_ul) - 0.0375*dth_ll**2*math.sin(th_ll - th_ul) +
        0.075*dth_lr**2*math.sin(th_lr - th_ur) + 3.67875 *
        math.sin(th_ul) - 7.3575*math.sin(th_ur),
    )
    return current_input


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

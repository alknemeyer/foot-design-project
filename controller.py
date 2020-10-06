"""
>>> %load_ext autoreload
>>> %autoreload 2
"""
# %run scripts/config_motors.py

##
import odrive
odrv0 = odrive.find_any()
assert odrv0 is not None, "Couldn't find the odrive"

##
import time
from typing import Literal
from scripts import lib
from scripts.lib import PositionControl

def l_dir(x: float): return -x
def r_dir(x: float): return -x


##
# >>> lib.foot_xy_to_th_deg(0,-0.417)
# (-85.99708257276197, -93.69841734138369) ## hmmm
ALMOST_STRAIGHT_DOWN_POS = (
    -500, #odrv0.axis0.encoder.pos_estimate,
    -2000, #odrv0.axis1.encoder.pos_estimate,
)

# zero deg = right
th0, th1 = lib.foot_xy_to_th_deg(0, -0.417)
ZERO_DEG_POS = (
    ALMOST_STRAIGHT_DOWN_POS[0] - r_dir(lib.leg_angle_deg_to_encoder_count(th0)),
    ALMOST_STRAIGHT_DOWN_POS[1] - l_dir(lib.leg_angle_deg_to_encoder_count(th1)),
)
# above is a more accurate verion of:
# ZERO_DEG_POS = (
#     ALMOST_STRAIGHT_DOWN_POS[0] - lib.encoder_cpr//2,
#     ALMOST_STRAIGHT_DOWN_POS[1] - lib.encoder_cpr//2,
# )
del ALMOST_STRAIGHT_DOWN_POS, th0, th1

## simple tests:
# just increment:
# with PositionControl(odrv0):
#     lib.slow_gains(odrv0)
#     while True:
#         odrv0.axis0.controller.pos_setpoint -= 100
#         odrv0.axis1.controller.pos_setpoint += 100
#         time.sleep(1)

# test for slipping:
# while True:
#     p0, p1 = odrv0.axis0.encoder.pos_estimate, odrv0.axis1.encoder.pos_estimate
#     print(f'{p0:7.3f}, {p1:7.3f}, {p0-p1:7.3f}')
#     time.sleep(1)



def set_to_nearest_angle(th: float, motornum: int, vel_limit: float):
    ax = odrv0.axis0 if motornum == 1 else odrv0.axis1

    curr = ax.encoder.pos_estimate
    # curr = ax.controller.pos_setpoint
    dest = ZERO_DEG_POS[motornum] + r_dir(lib.leg_angle_deg_to_encoder_count(th))
    dest_nearest = lib.nearest_cpr(curr=curr, dest=dest)

    print(f'{th=} {curr=} {dest=} {dest_nearest=}')
    # ax.controller.pos_setpoint = dest_nearest

    # https://github.com/madcowswe/ODrive/blob/fw-v0.4.12/docs/getting-started.md#trajectory-control
    deg2count = lib.leg_angle_deg_to_encoder_count
    ax.trap_traj.config.vel_limit = deg2count(vel_limit)
    ax.trap_traj.config.accel_limit = deg2count(100)
    ax.trap_traj.config.decel_limit = deg2count(100)
    ax.controller.move_to_pos(dest_nearest)


with PositionControl(odrv0):
    lib.fast_gains(odrv0)
    for x, y in ((0, -0.4), (0.2, -0.2), (-0.2, -0.2), (0, -0.2)):
        th0, th1 = lib.foot_xy_to_th_deg(x_m=x, y_m=y)
        # set_to_nearest_angle(th0, motornum=0, vel_limit=100)
        # set_to_nearest_angle(th1, motornum=1, vel_limit=150)
        time.sleep(2)


# ax0 = left motor (when looking from front)
#     = right/front leg (same view)
#     decreasing setpoint (neg) -> clockwise

# ax1 = right motor (when looking from front)
#     = left/back leg (same view)
#     increasing setpoint (pos) -> clockwise

# fact: 1200 counts ~= 110 degrees


def set_foot_position(x_m: float, y_m: float, vel: Literal['slow','fast']):
    lib.slow_gains(odrv0) if vel == 'slow' else lib.fast_gains(odrv0)
    
    th0, th1 = lib.foot_xy_to_th_deg(x_m=x_m, y_m=y_m)
    set_to_nearest_angle(th0, motornum=0, vel_limit=15)
    set_to_nearest_angle(th1, motornum=1, vel_limit=15)
#     odrv0.axis0.controller.vel_setpoint = v


## let's try a jump
with PositionControl(odrv0):
    # go into crouch position
    print('crouching')
    lib.slow_gains(odrv0)
    set_foot_position(x_m=0, y_m=-0.150, vel='slow')
    time.sleep(5)
    
    # launch!
    print('launching')
    lib.fast_gains(odrv0)
    set_foot_position(x_m=0, y_m=-0.380, vel='fast')
    time.sleep(1)

    # _try_ to land
    print('landing')
    lib.slow_gains(odrv0)
    set_foot_position(x_m=0, y_m=-0.150, vel='slow')
    time.sleep(1)

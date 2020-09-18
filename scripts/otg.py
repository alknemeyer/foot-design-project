##
import odrive
from odrive.enums import (
    AXIS_STATE_CLOSED_LOOP_CONTROL, CTRL_MODE_POSITION_CONTROL,
    AXIS_STATE_IDLE,
)
odrv0 = odrive.find_any()

##
import time
def pos(x): return x
def neg(x): return -x

##
class PositionControl:
    def __init__(self, odrv):
        self.odrv = odrv

    def __enter__(self):
        for ax in [self.odrv.axis0, self.odrv.axis1]:
            ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            ax.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
            ax.controller.pos_setpoint = ax.encoder.pos_estimate

    def __exit__(self, exc_type, exc_val, exc_tb):
        for ax in [self.odrv.axis0, self.odrv.axis1]:
            ax.requested_state = AXIS_STATE_IDLE
        
        dump_errors(odrv0, clear=True)


##
with PositionControl(odrv0):
    # go into crouch position
    print(odrv0.axis0.encoder.pos_estimate, odrv0.axis1.encoder.pos_estimate)
    for _ in range(25):
        for ax, sign in [(odrv0.axis0, neg), (odrv0.axis1, pos)]:
            ax.controller.pos_setpoint += sign(50)

        time.sleep(0.1)
    
    time.sleep(1)
    print(odrv0.axis0.encoder.pos_estimate, odrv0.axis1.encoder.pos_estimate)


# 1200 counts ~= 110 degrees
# ax0 = left motor (when looking from front)
#     = right/front leg (same view)
#     decreasing setpoint (neg) -> clockwise

# ax1 = right motor (when looking from front)
#     = left/back leg (same view)
#     increasing setpoint (pos) -> clockwise

import odrive
from odrive.enums import (
    MOTOR_TYPE_HIGH_CURRENT,
    ENCODER_MODE_INCREMENTAL,
    AXIS_STATE_FULL_CALIBRATION_SEQUENCE,
    AXIS_STATE_IDLE,
)
from odrive.utils import dump_errors
from fibre.protocol import ChannelBrokenException


def config_motor(ax):
    # current limit in [A]
    ax.motor.config.current_lim = 40  # not sure!

    # velocity limit [counts/s]
    ax.controller.config.vel_limit = 4096*8  # not sure!

    # calibration current [A]
    # = continuous current when stationary
    ax.motor.config.calibration_current = 5  # not sure!

    # number of magnet poles in motor divided by two
    ax.motor.config.pole_pairs = 20

    # motor type
    ax.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT

    # encoder count per revolution [CPR]
    # = 4x the pulse per revolution [PPR]
    ax.encoder.config.cpr = 4*500  # 4*1024= other one, from sticker on encoder
    ax.encoder.config.mode = ENCODER_MODE_INCREMENTAL
    ax.encoder.config.use_index = True

    # calibration accuracy. not sure about this one
    ax.encoder.config.calib_range = 0.05

    # run full calibration sequence
    import time
    ax.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    while ax.current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)

    # set startup sequence
    ax.config.startup_motor_calibration = False
    ax.config.startup_encoder_index_search = False  # True #??????????
    ax.config.startup_encoder_offset_calibration = False  # True #??????????
    ax.config.startup_closed_loop_control = False  # ????????
    ax.config.startup_sensorless_control = False

    # save
    ax.requested_state = AXIS_STATE_IDLE
    ax.encoder.config.pre_calibrated = True
    ax.motor.config.pre_calibrated = True

    dump_errors(odrv0)


odrv0 = odrive.find_any()

# brake resistance [Ohm]
odrv0.config.brake_resistance = 10  # gold one, brown one w/purple wires is 5ohm

config_motor(odrv0.axis0)
config_motor(odrv0.axis1)

# save config in memory
odrv0.save_configuration()

try:
    odrv0.reboot()
except ChannelBrokenException:
    print('Lost connection because of reboot')

"""
Motor configuration and calibration script. Run as follows:

    python scripts/config_motors.py

but copy-pasting the code into a REPL is better if you're still
figuring parameters out, and want to re-run this multiple times
without rebooting/re-finding the device

For some parameters/numbers related to current etc, check out:
https://www.harrisaerial.com/wp-content/uploads/2016/12/U10-Plus-80KV-Data.jpg
found via
https://www.harrisaerial.com/product/t-motor-u10-plus-80kv-100kv-170kv/
"""
import odrive
from odrive.enums import (
    MOTOR_TYPE_HIGH_CURRENT,
    ENCODER_MODE_INCREMENTAL,
    AXIS_STATE_FULL_CALIBRATION_SEQUENCE,
    AXIS_STATE_IDLE,
)
from odrive.utils import dump_errors
from fibre.protocol import ChannelBrokenException

cpr = 4*500


def config_motor(ax: 'odrive.Axis'):
    # current limit in [A]
    ax.motor.config.current_lim = 15  # not sure!

    # how much the current can swing (I think)
    # change this if doing current control, and get
    # CURRENT_UNSTABLE_ERROR or something
    # ax.motor.config.current_lim_tolerance

    # velocity limit [counts/s]
    ax.controller.config.vel_limit = cpr*16  # not sure!

    # calibration current [A]
    # = continuous current when stationary
    ax.motor.config.calibration_current = 10  # not sure!

    # number of magnet poles in motor divided by two
    ax.motor.config.pole_pairs = 20

    # motor type
    ax.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT

    # encoder count per revolution [CPR]
    # = 4x the pulse per revolution [PPR]
    ax.encoder.config.cpr = cpr
    ax.encoder.config.mode = ENCODER_MODE_INCREMENTAL
    ax.encoder.config.use_index = True

    # calibration accuracy. not sure about this one
    ax.encoder.config.calib_range = 0.05

    # run full calibration sequence
    import time
    ax.encoder.config.zero_count_on_find_idx = True
    ax.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    while ax.current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)

    # set startup sequence
    # see https://docs.odriverobotics.com/api/odrive.axis.axisstate
    # despite not using the encoder's index channel, turning
    # off encoder index search made the ODrive not recognize the
    # encoders. Not sure why!
    ax.config.startup_motor_calibration = False
    ax.config.startup_encoder_index_search = True  # False
    ax.config.startup_encoder_offset_calibration = False
    ax.config.startup_closed_loop_control = False
    ax.config.startup_sensorless_control = False

    # save
    ax.requested_state = AXIS_STATE_IDLE
    ax.encoder.config.pre_calibrated = True
    ax.motor.config.pre_calibrated = True

    dump_errors(odrv0, True)


# reboot first, to make sure we don't save other random settings on the device
print('finding and rebooting odrive...')
odrv0 = odrive.find_any()
try:
    odrv0.reboot()
except ChannelBrokenException:
    pass

print('finding odrive...')
odrv0 = odrive.find_any()

# brake resistance [Ohm]
odrv0.config.brake_resistance = 10  # gold one, brown one w/purple wires is 5ohm

print('configuring motor 0...')
config_motor(odrv0.axis0)

print('configuring motor 1...')
config_motor(odrv0.axis1)

print('saving config in memory....')
odrv0.save_configuration()

print('rebooting...')
try:
    odrv0.reboot()
except ChannelBrokenException:
    print('Lost connection because of reboot')

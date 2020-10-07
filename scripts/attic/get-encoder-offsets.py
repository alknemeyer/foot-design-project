"""
    This part is a MAJOR mind bender... you have to keep in mind:
    - motor direction
    - gear ratio (argghh!! a full turn on the motor is 180 degrees for a leg!)
    - cpr (counts per revolution)
"""
import odrive
from odrive.enums import *
import time
from ..lib import gear_ratio, encoder_cpr


def find_encoder_offset(ax: 'odrive.Axis', motornum: int):
    assert motornum in (0, 1)

    if not ax.encoder.index_found:
        print('finding index...')
        ax.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
        while ax.current_state != AXIS_STATE_IDLE:
            time.sleep(0.1)

        if not ax.encoder.index_found:
            print("Couldn't find the index!")

    offset = ax.encoder.config.offset

    leg = 'right' if motornum == 0 else 'left'
    input(f'move the {leg} leg horizontally {leg}, then press enter ')
    # this is an ever-increasing number
    horizontal_pos = ax.encoder.pos_estimate
    horizontal_cpr = ax.encoder.pos_cpr
    horizontal_count = ax.encoder.count_in_cpr

    # vertical down
    input(f'now move the {leg} leg vertically down, then press enter ')
    vertical_pos = ax.encoder.pos_estimate
    vertical_cpr = ax.encoder.pos_cpr
    vertical_count = ax.encoder.count_in_cpr

    print(f'{offset=}, {horizontal_pos=}, {vertical_pos=}')
    print(f'{horizontal_count=}, {vertical_count=}')

    # the leg turned a quarter, so the difference between these two
    # should be roughly that (scaled by gear_ratio)
    diff = gear_ratio*encoder_cpr/4 - abs(horizontal_pos - vertical_pos)
    assert -50 < diff < 50, (
        f'diff was: {diff} - you might need to be more precise? '
        'Also, check the cpr + gear_ratio'
    )

    # a clockwise direction
    if motornum == 0 and vertical_pos < horizontal_pos:
        counterclockwise_is_pos = True
    elif motornum == 1 and vertical_pos > horizontal_pos:
        counterclockwise_is_pos = True
    else:
        counterclockwise_is_pos = False

    print('Counter clockwise leg rotation is pos:', counterclockwise_is_pos,
          'direction:', ax.motor.config.direction)

    if not counterclockwise_is_pos:
        print('be careful - counterclockwise is negative!')

    # ax.encoder.pos_cpr is strictly in the range (0, cpr-1)
    # ax.encoder.config.offset #- This should print a number, like -326 or 1364.
    # ax.motor.config.direction
    return horizontal_pos


print('finding odrive...')
odrv0 = odrive.find_any()
assert odrv0 is not None, "Couldn't find the odrive"

# ax = odrv0.axis0

print('-'*20)
find_encoder_offset(odrv0.axis0, 0)
print('-'*20)
find_encoder_offset(odrv0.axis1, 1)

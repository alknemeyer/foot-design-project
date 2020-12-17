"""
This script is NOT meant to be running via the command
line. Instead, activate the conda environment, launch
ipython:

    $ ipython

and then copy-paste in commands block by block. When
you feel that a certain block of code is working well,
copy it into the `lib.py` file

By default, if you update the `lib.py` file, the changes
won't reflect in a live IPython session. Change that by
copying in the following code:

    >>> %load_ext autoreload
    >>> %autoreload 2

See eg. https://alknemeyer.github.io/jupyter-notebook-workflow/#autoreloading-changed-code
"""

########################################## SET UP
import time
from scripts import lib
from scripts.lib import (
    PositionControl, set_foot_position as setfoot,
)
import odrive
odrv0 = odrive.find_any()

# move leg straight down (so the foot is below the axle,
# as far down as possible) then run the line of code below
# Doing so gets everything to a "known start position"
# Simple using the index lines of the encoders would be
# better, though!
lib.zero_motors_no_index_straightdown(odrv0)
odrv0

########################################## SIMPLE TESTS
from scripts import test
test.increment(odrv0, delta=100)  # + = bend knees
test.slipping(odrv0)
test.slow_position_control(odrv0)

########################################## INCREASE CURRENT LIMITS
odrv0.axis0.motor.config.current_lim = 20
odrv0.axis1.motor.config.current_lim = 20

odrv0.axis0.motor.config.current_lim_tolerance = 4
odrv0.axis1.motor.config.current_lim_tolerance = 4


########################################## SIMPLE LAUNCH
with PositionControl(odrv0):
    # lib.fast_gains(odrv0)
    lib.set_gains(odrv0, pos_scale=3, vel_scale=5, bandwidth_hz=10)
    th0, th1 = lib.foot_xy_to_th_deg(x_m=0, y_m=-0.35)
    lib.set_to_nearest_angle(odrv0, th0, motornum=0)#, vel_limit=180)
    lib.set_to_nearest_angle(odrv0, th1, motornum=1)#, vel_limit=180)
    time.sleep(2)

    lib.slow_gains(odrv0)
    th0, th1 = lib.foot_xy_to_th_deg(x_m=0, y_m=-0.15)
    lib.set_to_nearest_angle(odrv0, th0, motornum=0, vel_limit=180)
    lib.set_to_nearest_angle(odrv0, th1, motornum=1, vel_limit=180)
    time.sleep(2)


########################################## LOG REPEATED JUMPS
"""
You might need to adjust the permissions for any plugged in
sensors/the teensy. On Ubuntu, this will be something like:

    $ sudo chmod 666 /dev/ttyACM1

Or from the Python session:

    >>> import os
    >>> os.system('sudo chmod 666 /dev/ttyACM0')

Log the optoforce at the same time, in another terminal:

    $ python scripts/optoforce.py
    $ python scripts/plot_optoforce.py

BUT Alex suggests rather incorporating it into the
`log_while_delaying()` function, which would make the
data accessible to the control and make absolutely sure
that the readings are all synced up
"""
import csv
from datetime import datetime

# this requires the sensor-logger submodule to be downloaded
# and named as sensor_logger
from sensor_logger.scripts import receive_comms

# might need to adjust the port
# run this once only, since reconnecting could cause issues!
logger = receive_comms.TeensyLogger(port='/dev/ttyACM3')

# check that it reads without issue. We don't want the line
# to just hang in the control loop
logger.read()

data = []
experiment_start_datetime = datetime.now().strftime("%H-%M-%S")


def log_while_delaying(delay: float):
    tstart = time.time()
    while time.time() < tstart + delay:
        data.append([*logger.read(), *lib.upper_leg_angles_deg(odrv0)])


# go into crouch position, then launch, then try to land
with PositionControl(odrv0):
    while True:
        try:
            # crouch
            setfoot(odrv0, x_m=0, y_m=-0.15, gains='medium', vel_limit=360)
            log_while_delaying(2.)

            # launch
            lib.set_gains(odrv0, pos_scale=5, vel_scale=5, bandwidth_hz=10)
            setfoot(odrv0, x_m=0, y_m=-0.38)#, vel_limit=360)
            log_while_delaying(0.4)

            # impact
            setfoot(odrv0, x_m=0, y_m=-0.2, gains='medium', vel_limit=360)
            log_while_delaying(2.)

        except KeyboardInterrupt:
            break


with open(f'data/leg-data-{experiment_start_datetime}.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    header = receive_comms.PACKET_HEADER + ['th_ul [deg]', 'th_ur [deg]', 'dth_ul [deg/s]', 'dth_ur [deg/s]']
    writer.writerow(header)
    writer.writerows(data)


########################################## ATTIC


# from optoforce import OptoForce16 as OptoForce
# def read_foot_forces_2D(force_sensor: OptoForce):
#     reading = force_sensor.read(only_latest_data=True)
#     return {'L_x': reading.Fy, 'L_y': reading.Fz}

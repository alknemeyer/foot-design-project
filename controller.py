"""
$ python scripts/config_motors.py
$ ipython
>>> %load_ext autoreload
>>> %autoreload 2
"""

##
import odrive
odrv0 = odrive.find_any()
assert odrv0 is not None, "Couldn't find the odrive"

##
import time
from scripts import lib
from scripts.lib import PositionControl, set_foot_position as setfoot

import logging  # for optoforce. Nothing else logs
logging.basicConfig(
    format='%(asctime)s | %(levelname)s | %(name)s: %(message)s',
    datefmt='%I:%M:%S',
    level=logging.INFO,
)

## simple tests:
# just increment:
with PositionControl(odrv0):
    lib.medium_gains(odrv0)
    for _ in range(5):
        odrv0.axis0.controller.pos_setpoint -= 100
        odrv0.axis1.controller.pos_setpoint += 100
        time.sleep(1)
        print('.'*50)

# test for slipping:
# while True:
#     p0, p1 = odrv0.axis0.encoder.pos_estimate, odrv0.axis1.encoder.pos_estimate
#     print(f'{p0:7.3f}, {p1:7.3f}, {p0-p1:7.3f}')
#     time.sleep(1)

# with PositionControl(odrv0):
#     lib.fast_gains(odrv0)
#     for x, y in ((0, -0.4), (0.2, -0.2), (-0.2, -0.2), (0, -0.2)):
#         th0, th1 = lib.foot_xy_to_th_deg(x_m=x, y_m=y)
#         print(f'changing to: {th0=} {th1=} {x=} {y=}')
#         set_to_nearest_angle(th0, motornum=0, vel_limit=180)
#         set_to_nearest_angle(th1, motornum=1, vel_limit=180)
#         time.sleep(5)


## let's try a jump
# go into crouch position, then launch, then try to land
with PositionControl(odrv0):
    # while True:
    for i in [50, 100, 150, 200, 250, 300]:
        print('crouching')
        setfoot(odrv0, x_m=0, y_m=-0.150, gains='slow', vel_limit=360, sleep=2)
        
        print('launching: i =', i)
        setfoot(odrv0, x_m=0, y_m=-0.380, gains='fast', vel_limit=i*360, sleep=0.5)
        
        print('landing')
        setfoot(odrv0, x_m=0, y_m=-0.150, gains='fast', vel_limit=3*360, sleep=2)


##
from optoforce import OptoForce16 as OptoForce
from math import isclose
with PositionControl(odrv0), OptoForce() as force_sensor:
    read_fz = lambda: force_sensor.read(only_latest_data=True).Fz

    # while True:
    for i in [50, 100, 150, 200, 250, 300, 350, 400]:
        print('crouching')
        setfoot(odrv0, x_m=0, y_m=-0.150, gains='medium', vel_limit=360, sleep=2)
        fzcrouch = read_fz()
        
        print('launching')
        setfoot(odrv0, x_m=0, y_m=-0.380, gains='fast', vel_limit=i*360)
        # push off until there's no force on the ground
        while read_fz() >= fzcrouch:
            # print(f'{fzcrouch=} {read_fz()=}')
            pass
        
        # now we're airborne

        print('landing')
        setfoot(odrv0, x_m=0, y_m=-0.150, gains='medium', vel_limit=3*360)
        fzprev = read_fz()
        while True:
            time.sleep(0.1)
            fzcurr = read_fz()
            if isclose(fzprev, fzcurr, abs_tol=10):
                break
            else:
                fzprev = fzcurr

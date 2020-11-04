import time
from scripts import lib
from scripts.lib import (
    PositionControl, CurrentControl, set_foot_position as setfoot,
)
import odrive
odrv0 = odrive.find_any()
assert odrv0 is not None, "Couldn't find the odrive"
lib.zero_motors_no_index(odrv0)


## simple tests:
# just increment:
with PositionControl(odrv0):
    lib.medium_gains(odrv0)
    for _ in range(3):
        odrv0.axis0.controller.pos_setpoint -= 100
        odrv0.axis1.controller.pos_setpoint += 100
        time.sleep(1)
        print('.'*50)

# test for slipping:
while True:
    try:
        p0, p1 = odrv0.axis0.encoder.pos_estimate, odrv0.axis1.encoder.pos_estimate
        print(f'{p0:7.3f}, {p1:7.3f}, {p0-p1:7.3f}')
        time.sleep(1)
    except KeyboardInterrupt:
        break

# with PositionControl(odrv0):
#     lib.fast_gains(odrv0)
#     for x, y in ((0, -0.4), (0.2, -0.2), (-0.2, -0.2), (0, -0.2)):
#         th0, th1 = lib.foot_xy_to_th_deg(x_m=x, y_m=y)
#         print(f'changing to: {th0=} {th1=} {x=} {y=}')
#         lib.set_to_nearest_angle(odrv0, th0, motornum=0, vel_limit=180)
#         lib.set_to_nearest_angle(odrv0, th1, motornum=1, vel_limit=180)
#         time.sleep(5)

odrv0.axis0.motor.config.current_lim = 20
odrv0.axis1.motor.config.current_lim = 20

odrv0.axis0.motor.config.current_lim_tolerance = 40
odrv0.axis1.motor.config.current_lim_tolerance = 40

# check that it can balance
with PositionControl(odrv0):
    lib.set_gains(odrv0, pos_scale=10, vel_scale=10, bandwidth_hz=5)
    setfoot(odrv0, x_m=-0.02, y_m=-0.350, vel_limit=100_00*360, sleep=1)
# with PositionControl(odrv0):
#     lib.set_gains(odrv0, pos_scale=15, vel_scale=10, bandwidth_hz=30)
#     setfoot(odrv0, x_m=-0.02, y_m=-0.380, vel_limit=None, sleep=1)


## let's try a jump
# go into crouch position, then launch, then try to land
with PositionControl(odrv0):
    # while True:
    for i in [50, 100, 150, 200, 250, 300]:
        print('crouching')
        setfoot(odrv0, x_m=0, y_m=-0.150, gains='medium', vel_limit=360, sleep=2)
        
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

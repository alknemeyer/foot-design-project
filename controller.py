########################################## OPTIONAL STUFF
# ipython specific
%load_ext autoreload  # type: ignore
%autoreload 2         # type: ignore

# give force sensor required permissions
import os
os.system('sudo chmod 666 /dev/ttyACM0')

########################################## SET UP
import time
from scripts import lib
from scripts.lib import (
    PositionControl, CurrentControl, set_foot_position as setfoot,
)
import odrive
odrv0 = odrive.find_any()
lib.zero_motors_no_index_straightdown(odrv0)
odrv0

########################################## SIMPLE TESTS
from scripts import test
test.increment(odrv0)
test.slipping(odrv0)
test.slow_position_control(odrv0)

########################################## 
# for ax in (odrv0.axis0, odrv0.axis1):
#     ax.motor.config.current_lim = 15
#     ax.motor.config.current_lim_tolerance = 3
#     # default:
#     ax.motor.config.current_lim = 15
#     ax.motor.config.current_lim_tolerance = 1.25

kp = 0.2
kd = kp/10
th_ul, th_ur, dth_ul, dth_ur = lib.upper_leg_angles(odrv0)
controller0 = lib.PDControl(th_ur, 0, kp, kd)  # -40 to launch
controller1 = lib.PDControl(th_ul, 0, kp, kd)  # +40 to launch

with CurrentControl(odrv0):
    while True:
        try:
            th_ul, th_ur, dth_ul, dth_ur = lib.upper_leg_angles(odrv0)
            curr0 = controller0(th_ur, dth_ur)
            curr1 = controller1(th_ul, dth_ul)
            lib.set_current(odrv0, motor0=curr0, motor1=curr1)
        except KeyboardInterrupt:
            break

#
from scripts.controller_generated import impedance_control, foot_state_polar
th_ul, th_ur, dth_ul, dth_ur = lib.upper_leg_angles(odrv0)
r, th, dr, dth = foot_state_polar(th_ul, th_ur, dth_ul, dth_ur)
ddr_controller = lib.PDControl(r, dr, kp=0.1, kd=0.01)
ddth_controller = lib.PDControl(th, dth, kp=0.1, kd=0.01)

looptimes = []
with CurrentControl(odrv0):
    while True:
        try:
            th_ul, th_ur, dth_ul, dth_ur = lib.upper_leg_angles(odrv0)
            r, th, dr, dth = foot_state_polar(th_ul, th_ur, dth_ul, dth_ur)
            curr0, curr1 = impedance_control(
                ddr_setpoint=ddr_controller(r, dr),
                ddth_setpoint=ddth_controller(th, dth),
                th_ul=th_ul, th_ur=th_ur, dth_ul=dth_ul, dth_ur=dth_ur,
                L_x=0, L_y=0, Kt=lib.torque_constant,
            )
            lib.set_current(odrv0, motor0=curr0, motor1=curr1)

            looptimes.append(time.time())
        except KeyboardInterrupt:
            break

#
odrv0.axis0.motor.config.current_lim = 20
odrv0.axis1.motor.config.current_lim = 20

odrv0.axis0.motor.config.current_lim_tolerance = 40
odrv0.axis1.motor.config.current_lim_tolerance = 40

#
with PositionControl(odrv0):
    # lib.fast_gains(odrv0)
    lib.set_gains(odrv0, pos_scale=3, vel_scale=5, bandwidth_hz=10)
    th0, th1 = lib.foot_xy_to_th_deg(x_m=-0.01, y_m=-0.4)
    lib.set_to_nearest_angle(odrv0, th0, motornum=0)#, vel_limit=180)
    lib.set_to_nearest_angle(odrv0, th1, motornum=1)#, vel_limit=180)
    time.sleep(2)

# check that it can balance
with PositionControl(odrv0):
    # lib.set_gains(odrv0, pos_scale=2, vel_scale=2, bandwidth_hz=5)
    setfoot(odrv0, x_m=0, y_m=-0.350, vel_limit=100_00*360, sleep=1, gains='fast')
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


# from optoforce import OptoForce16 as OptoForce
# def read_foot_forces_2D(force_sensor: OptoForce):
#     reading = force_sensor.read(only_latest_data=True)
#     return {'L_x': reading.Fy, 'L_y': reading.Fz}


# def set_foot_force(odrv: 'odrive.ODrive', L_x: float, L_y: float):
#     state = read_leg_state(odrv)
#     lib.current_control(L_x, L_y, **state, Kt=0.08)


# def current_jump(odrv0: 'odrive.ODrive', current: float, until_angle_deg: float):
#     lib.set_current(odrv0, motor0=current, motor1=current)
#     while lib.encoder_count_to_leg_angle_deg() < until_angle_deg:
#         pass

##
# from math import isclose
# with PositionControl(odrv0), OptoForce() as force_sensor:
#     read_fz = lambda: force_sensor.read(only_latest_data=True).Fz

#     # while True:
#     for i in [50, 100, 150, 200, 250, 300, 350, 400]:
#         print('crouching')
#         setfoot(odrv0, x_m=0, y_m=-0.150, gains='medium', vel_limit=360, sleep=2)
#         fzcrouch = read_fz()
        
#         print('launching')
#         setfoot(odrv0, x_m=0, y_m=-0.380, gains='fast', vel_limit=i*360)
#         # push off until there's no force on the ground
#         while read_fz() >= fzcrouch:
#             # print(f'{fzcrouch=} {read_fz()=}')
#             pass
        
#         # now we're airborne

#         print('landing')
#         setfoot(odrv0, x_m=0, y_m=-0.150, gains='medium', vel_limit=3*360)
#         fzprev = read_fz()
#         while True:
#             time.sleep(0.1)
#             fzcurr = read_fz()
#             if isclose(fzprev, fzcurr, abs_tol=10):
#                 break
#             else:
#                 fzprev = fzcurr

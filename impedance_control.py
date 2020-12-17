"""
For general documentation, rather read through `controller.py`
first
"""
from scripts import lib
from scripts.lib import CurrentControl
import odrive
odrv0 = odrive.find_any()


########################################## INCREASE CURRENT
## you MIGHT want to increase the current available
## to each motor:
# for ax in (odrv0.axis0, odrv0.axis1):
#     ax.motor.config.current_lim = 15
#     ax.motor.config.current_lim_tolerance = 3


########################################## MOTOR ANGLE IMPEDANCE CONTROL
kp = 0.2
kd = kp/10
th_ul, th_ur, dth_ul, dth_ur = lib.upper_leg_angles_deg(odrv0)
controller0 = lib.PDControl(th_ur, 0, kp, kd)  # -40 to launch
controller1 = lib.PDControl(th_ul, 0, kp, kd)  # +40 to launch

with CurrentControl(odrv0):
    while True:
        try:
            th_ul, th_ur, dth_ul, dth_ur = lib.upper_leg_angles_deg(odrv0)
            curr0 = controller0(th_ur, dth_ur)
            curr1 = controller1(th_ul, dth_ul)
            lib.set_current(odrv0, motor0=curr0, motor1=curr1)
        except KeyboardInterrupt:
            break


########################################## LEG POSITION IMPEDANCE CONTROL
# TODO: what is 0 deg? SHOULD be pointing right!
#       also: radians vs degrees?
#       also: positive vs negative angles/current??
from scripts import controller_generated as cg

th_ul, th_ur, dth_ul, dth_ur = lib.upper_leg_angles(odrv0)  # radians

r, th, dr, dth = cg.foot_state_polar(th_ul, th_ur, dth_ul, dth_ur, lib.l3x, lib.l3y)

ddr_controller = lib.PDControl(r, dr, kp=0.01, kd=0.001)
ddth_controller = lib.PDControl(th, dth, kp=0.01, kd=0.001)

with CurrentControl(odrv0):
    while True:
        try:
            th_ul, th_ur, dth_ul, dth_ur = lib.upper_leg_angles(odrv0)  # rads
            r, th, dr, dth = cg.foot_state_polar(th_ul, th_ur, dth_ul, dth_ur, lib.l3x, lib.l3y)
            if th_ul == th_ur:  # getting weird errors if the angles are the same?
                th_ul += 0.001
            try:
                curr0, curr1 = cg.impedance_control(
                    ddr_setpoint=ddr_controller(r, dr),
                    ddth_setpoint=ddth_controller(th, dth),
                    th_ul=th_ul, th_ur=th_ur, dth_ul=dth_ul, dth_ur=dth_ur,
                    L_x=0, L_y=0, l3x=lib.l3x, l3y=lib.l3y, Kt=lib.torque_constant,
                )
                lib.set_current(odrv0, motor0=-5*curr0, motor1=0)
                print('current:', curr0, curr1)
            except ZeroDivisionError:
                print('error!')
        except KeyboardInterrupt:
            break
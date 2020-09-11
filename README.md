# ODrive documentation etc

<!-- source-conda && conda activate foot-design && odrivetool -->

## Quick doc links:
* [ODrive getting started documentation](https://docs.odriverobotics.com/)
* [ascii-protocol](https://docs.odriverobotics.com/ascii-protocol.html)
* [ODriveArduino.cpp](https://github.com/madcowswe/ODrive/blob/master/Arduino/ODriveArduino/ODriveArduino.cpp)

## Set up environment

### Python
Install conda or miniconda, then:

```bash
$ conda create --name foot-design
$ conda activate foot-design
$ conda install pip
$ python -m pip install odrive
# there might be more install instructions for your
# particular OS: https://docs.odriverobotics.com/
# eg. on ubuntu, I have to install "udev rules"

# after you've done that, plug in the ODrive, power it
# externally (ie >=24V from a bench power supply) and
# then enter:
$ odrivetool

# on my laptop, a few lines of errors get displayed saying
# "[stuff] was never awaited", but after ~10 seconds you
# should see something like:
# "Connected to ODrive 20603595524B as odrv0"
# in cyan text. If so -- great!
# you'll start every session by activating your conda env
# and then entering `odrivetool`
```

If using vs code, you'll need to update the `""python.pythonPath"` setting in `.vscode/settings.json` to your actual virtual environment

### Arduino
Next up, we need to install arduino and patch it to run teensy code. Follow the instructions on the [teensy website](https://www.pjrc.com/teensy/td_download.html). If you're running linux, there is a ridiculously simple install process under the heading "Command Line Install" (you can skip the last two lines about changing directory and running `make`). I put all those files in a folder called `Arduino`. If you're using vs code, you'll need to update the `includePath` in `.vscode/c_cpp_properties.json`

Next up, test that that worked:
1. plug in the teensy
2. launch arduino
3. Tools > board > Teensyduino > Teensy 4.0
4. File > Examples > 0.1.Basics > Blink
5. Click "Upload"

If you can modify the sketch to various blink rates and upload it, you're almost done!

I also install the arduino extension from Microsoft, which can be installed quickly by launching VS Code Quick Open (Ctrl + P), typing `ext install vscode-arduino`, and then pressing enter.

Finally, install ODrive arduino as an arduino library, as described on their [github page](https://github.com/madcowswe/ODrive/tree/devel/Arduino/ODriveArduino). To get the files, you'll need to clone their repository:

```bash
$ git clone https://github.com/madcowswe/ODrive.git
```

Afterwards, open [hopper_code/hopper.ino](hopper_code/hopper.ino) and click `upload`. If you get an error saying something like `fatal error: ODriveArduino.h: No such file or directory`, then you didn't install the ODrive arduino library properly

## The robot

### Equipment used

| Item             | Name                                                                |
| ---------------- | ------------------------------------------------------------------- |
| Motor controller | [ODrive](https://odriverobotics.com/)                               |
| Motor            | [T-motor U-10 80rpm/V](http://store-en.tmotor.com/goods.php?id=362) |
| Encoder          | [E6B2-CWZ3E](https://www.ia.omron.com/data_pdf/cat/e6b2-c_ds_e_6_1_csm491.pdf) |
|                  |                                                                     |

### Setting up parameters with python:
Values from T-motor link above. Plug in the motors before entering entering the code below

```python
def config_motor(ax):
    # current limit in [A]
    ax.motor.config.current_lim = 40 # not sure!

    # velocity limit [counts/s]
    ax.controller.config.vel_limit = 4096*8 # not sure!

    # calibration current [A]
    # = continuous current when stationary
    ax.motor.config.calibration_current = 5  # not sure!

    # number of magnet poles in motor divided by two
    ax.motor.config.pole_pairs = 20

    # motor type
    ax.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT

    # encoder count per revolution [CPR]
    # = 4x the pulse per revolution [PPR]
    ax.encoder.config.cpr = 4*500 # 4*1024= other one, from sticker on encoder
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
    ax.config.startup_encoder_index_search = False#True #??????????
    ax.config.startup_encoder_offset_calibration = False#True #??????????
    ax.config.startup_closed_loop_control = False  # ????????
    ax.config.startup_sensorless_control = False

    # save
    ax.requested_state = AXIS_STATE_IDLE
    ax.encoder.config.pre_calibrated = True
    ax.motor.config.pre_calibrated = True

    dump_errors(odrv0)

# brake resistance [Ohm]
odrv0.config.brake_resistance = 10  # gold one, brown one w/purple wires is 5ohm

config_motor(odrv0.axis0)
config_motor(odrv0.axis1)

# save config in memory
odrv0.save_configuration()
odrv0.reboot()
```

It may be useful to live plot the setpoint vs motor output. Do so using,
```python
start_liveplotter(lambda:[odrv0.axis0.encoder.pos_estimate,
                          odrv0.axis0.controller.pos_setpoint])
```

[Tuning](https://docs.odriverobotics.com/control.html#Tuning)
```python
# setup
ax = odrv0.axis0
ax.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
# wait...
ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
# use ax.requested_state = AXIS_STATE_IDLE to stop vibrating etc

# defaults
ax.controller.config.pos_gain = 20.0 # [(counts/s) / counts]
ax.controller.config.vel_gain = 5.0 / 10000.0 # [A/(counts/s)]
ax.controller.config.vel_integrator_gain = 10.0 / 10000.0 # [A/((counts/s) * s)]

ax.controller.config.vel_integrator_gain = 0

# increase vel_gain until it's unstable, then request idle state
ax.controller.config.vel_gain = 8.0 / 10000.0   # vibrates at 16.0

# increase pos_gain until it's unstable, then bring back to smoothness
ax.controller.config.pos_gain = 80.0     # should be 120.0

# integrator gain
def set_int_gain(bandwidth_hz):
    ax = odrv0.axis0
    vel_gain = ax.controller.config.vel_gain
    ax.controller.config.vel_integrator_gain = 0.5 * bandwidth_hz * vel_gain
set_int_gain(10)

odrv0.save_configuration()
odrv0.reboot()
```

After configuring the motor, do some position control:
```python
import time
ax = odrv0.axis0
ax = odrv0.axis1

# for encoders without an index signal
# ax.requested_state = (
#     AXIS_STATE_ENCODER_OFFSET_CALIBRATION
# )
# while ax.current_state != AXIS_STATE_IDLE:
#     time.sleep(0.1)

# for encoders with an index signal
ax.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
while ax.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)

dump_errors(odrv0)
#odrv0.axis0.error   # should be 0
#odrv0.axis0.encoder.config.offset  # should print a number
#odrv0.axis0.motor.config.direction  # should print 1 or -1

ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
ax.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
dump_errors(odrv0)

zero = ax.encoder.config.offset
ax.controller.pos_setpoint = zero + 250
time.sleep(1)
ax.controller.pos_setpoint = zero + 500
time.sleep(1)
ax.controller.pos_setpoint = zero + 750
time.sleep(1)
ax.controller.pos_setpoint = zero
time.sleep(1)

ax.requested_state = AXIS_STATE_IDLE
```

```python
# a full test
import time
for ax in [odrv0.axis0, odrv0.axis1]:
    ax.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
    while ax.current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)
dump_errors(odrv0)

for ax in [odrv0.axis0, odrv0.axis1]:
    ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    ax.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
dump_errors(odrv0)

import math, time, sys
i = 0
while True:
    i += 1
    pt = math.fabs(200*math.sin(math.radians(10*i)))
    for ax in [odrv0.axis0, odrv0.axis1]:
        offset = ax.encoder.config.offset
        ax.controller.pos_setpoint = offset + pt
    
    #print((pt, i), end=' '); sys.stdout.flush()
    print('.', end=''); sys.stdout.flush()
    time.sleep(0.1)
```

### Optional nicer print for `odrv0.reboot()`:
```python
from fibre.protocol import ChannelBrokenException
try:
    odrv0.reboot()
except ChannelBrokenException:
    print('Lost connection because of reboot')
```

### Tip for debugging:
From the [ODrive troubleshooting](https://docs.odriverobotics.com/troubleshooting) page:

```python
>>> dump_errors(odrv0)       # show errors
>>> dump_errors(odrv0, True) # show errors and then clear em
```

## Interfacing with arduino:

[Folder in Github repo](https://github.com/madcowswe/ODrive/tree/master/Arduino/ODriveArduino)

## Setting up encoder

| Wire   | Function            |
| ------ | ------------------- |
| Brown  | Input power (5-12V) |
| Black  | Phase A output      |
| White  | Phase B output      |
| Orange | Phase Z output      |
| Blue   | 0V                  |
| Shield | GND                 |


## Firmware upgrade issues:

* [Issues with Device Firmware Upgrade (DFU)](https://github.com/madcowswe/ODrive/issues/179)
* Related: [ValueError: The device has no langid](https://github.com/pyusb/pyusb/issues/139). Fix is to run as superuser


## CAN interface:

* [Announcement thread](https://discourse.odriverobotics.com/t/can-interface-available-for-testing/1448)
* [Github fork](https://github.com/Wetmelon/ODrive/tree/feature/CAN)
* [Protocol (more likely up to date than thread?)](https://github.com/Wetmelon/ODrive/blob/feature/CAN/docs/can-protocol.md)

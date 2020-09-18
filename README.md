# ODrive documentation etc

## TODO:
* need to remember how to get trajectory optimization stuff set up + document that

<!-- source-conda && conda activate foot-design && odrivetool -->

## Quick doc links:
* [ODrive getting started documentation](https://docs.odriverobotics.com/)
* [ascii-protocol](https://docs.odriverobotics.com/ascii-protocol.html)
* [ODriveArduino.cpp](https://github.com/madcowswe/ODrive/blob/master/Arduino/ODriveArduino/ODriveArduino.cpp)

## Software environment setup

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

Afterwards, open [./controller/hopper_code.ino](./controller/hopper_code.ino) and click `upload`. If you get an error saying something like `fatal error: ODriveArduino.h: No such file or directory`, then you didn't install the ODrive arduino library properly

## The robot

### Equipment used

| Item             | Name                                                                |
| ---------------- | ------------------------------------------------------------------- |
| Microcontroller  | [teensy 4.0](https://www.pjrc.com/store/teensy40.html)              |
| Motor controller | [ODrive](https://odriverobotics.com/)                               |
| Motor x2         | [T-motor U-10 80rpm/V](http://store-en.tmotor.com/goods.php?id=362) |
| Motor encoder x2 | [HEDL-5640-A13](https://www.mouser.co.za/ProductDetail/Broadcom-Avago/HEDL-5640A13?qs=RuhU64sK2%252Bv3nPu5sOD%2FhQ==) |
| Force sensor     | [OptoForce OMD-45-FH-2000N](http://schlu.com/pdf/Optoforce_Sensore_di_forza_3D_OMD-45-FH-2000N_1.5_EN.pdf) |
<!-- | Encoder          | [E6B2-CWZ3E](https://www.ia.omron.com/data_pdf/cat/e6b2-c_ds_e_6_1_csm491.pdf) | -->

### Wiring things together

The general setup is:
* laptop <-> teensy via USB connection
* teensy <-> ODrive via two Rx/Tx wires [%]
* ODrive <-> motor encoders via premade connectors. See below
* ODrive <-> motors. Each motor has three thick wires which must be directly connected to the ODrive
* ODrive <-> shunt resistor, plugged into ODrive "AUX" port. I can't remember how to do the calculation for the required resistance/power, unfortunately!
* laptop <-> OptoForce via USB connection
<!-- * teensy <-> yaw/pitch encoder via four A/B encoder wires, passing through a logic level converter [%] (not used while on vertical rail -- only boom) -->

[%] = see [./controller/hopper_code.ino](./controller/hopper_code.ino) for pin numbers. I don't want to document them here and then have things go out of sync

#### ODrive <-> motor encoders
The encoder has pins as follows:

| pin | type   | | pin | type |
|-----|--------|-|-----|------|
| 1   | NC     | | 2   | +5V  |
| 3   | GND    | | 4   | NC   |
| 5   | not(A) | | 6   | A    |
| 7   | not(B) | | 8   | B    |
| 9   | not(I) | | 10  | I    |

where NC = No Connection, and I = Index = Z

When looking at the connector, they correspond to the diagram below, where crosses over numbers indicates pins which aren't connected:

<img src="diagrams/motor-encoder-wiring.jpeg" alt="motor encoder wiring diagram" width="200" style="display:block; margin:auto;"/>

_Fig. 1_

Now, the connection works as follows:

<img src="diagrams/motor-encoder-odrive-wiring.jpeg" alt="photo of connection between motor encoder and ODrive" width="400" style="display:block; margin:auto;"/>

_Fig. 2_

Note that the second line from the left (ground, as shown by Fig. 1) crosses over the lines and ends up at the ground pin of the ODrive. When looking at the robot from the front, the left motor is motor 0 (M0)

<!-- #### Setting up encoder

| Wire   | Function            |
| ------ | ------------------- |
| Brown  | Input power (5-12V) |
| Black  | Phase A output      |
| White  | Phase B output      |
| Orange | Phase Z output      |
| Blue   | 0V                  |
| Shield | GND                 | -->

### Setting up parameters with python:
Values from T-motor link above. Plug in the motors before running the code in `scripts/config_motors.py`. Do either by copy-pasting the code into the `odrivetool` REPL, or just by running:

```bash
$ python scripts/config_motors.py
```

### Live plotting
It may be useful to live plot the setpoint vs motor output. Do so using,
```python
start_liveplotter(lambda:[odrv0.axis0.encoder.pos_estimate,
                          odrv0.axis0.controller.pos_setpoint,
                          odrv0.axis1.encoder.pos_estimate,
                          odrv0.axis1.controller.pos_setpoint])
```
You don't need to run that in a new terminal or anything - it runs in the background. Press enter if you don't see the `In [x] :` prompt

### [Tuning](https://docs.odriverobotics.com/control.html#Tuning)

Run the following code by copy-pasting it into the `odrivetool` REPL, line by line:

```python
# set ax to odrv0.axis0 or axis1
ax = odrv0.axis0
# ax = odrv0.axis1

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

## Firmware upgrade issues:

* [Issues with Device Firmware Upgrade (DFU)](https://github.com/madcowswe/ODrive/issues/179)
* Related: [ValueError: The device has no langid](https://github.com/pyusb/pyusb/issues/139). Fix is to run as superuser


## CAN interface:

* [Announcement thread](https://discourse.odriverobotics.com/t/can-interface-available-for-testing/1448)
* [Github fork](https://github.com/Wetmelon/ODrive/tree/feature/CAN)
* [Protocol (more likely up to date than thread?)](https://github.com/Wetmelon/ODrive/blob/feature/CAN/docs/can-protocol.md)

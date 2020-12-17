# ODrive "knowledge"
This doc summarizes tips for using the ODrive which can be found in the official documentation, but have been useful for this project, so have been copied here for discoverability


### Tuning
See the [tuning guide](https://docs.odriverobotics.com/control.html#Tuning). The process used for this robot is as follows:

Copy-paste the folloing code into the `odrivetool` REPL, line by line:

```python
# set ax to odrv0.axis0 or axis1. Each motor *should* require
# the same gains
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
    vel_gain = ax.controller.config.vel_gain
    ax.controller.config.vel_integrator_gain = 0.5 * bandwidth_hz * vel_gain

set_int_gain(10)

odrv0.save_configuration()
odrv0.reboot()
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


### Tip for debugging:
From the [ODrive troubleshooting](https://docs.odriverobotics.com/troubleshooting) page:

```python
# show errors
>>> dump_errors(odrv0)

# show errors and then clear em. This is useful if an
# error occurs, you fix it, and then need to manually
# clear error flags before the ODrive works again
>>> dump_errors(odrv0, True)
```


### Optional nicer print for `odrv0.reboot()`:
When the odrive reboots, it raises an error in your Python code. Do this too often and you'll stop noticing real errors. A better approach is to do the following, which specifically catches the error thrown by a reboot:

```python
from fibre.protocol import ChannelBrokenException
try:
    odrv0.reboot()
except ChannelBrokenException:
    print('Lost connection because of reboot')
```
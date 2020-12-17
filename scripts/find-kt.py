"""
Code to find the torque constant Kt of the motors.
If you can find this by documentation -- great! No need
for this!

_Otherwise_, the setup is as follows:

1. remove the two segments of the leg which connect to
   form a foot

     | axle |
       //\\
      //  \\
     //    \\
     \\    //
      \\  //  <-- remove these links
       \\//

2. place the force sensor (or even a scale) under one
   of the remaining upper links, so that a (positive)
   input current results in a measureable force:

     | axle |
       //\\
      //  \\
     //    \\
          -----  <-- force/weight measurement device

   Be careful with units! Eg. "measuring mass" (kg)
   vs force vs torque! There may be a bug in this
   script!

3. follow the instructions below. Run the commands
   in this file by copy-pasting them into an IPython
   terminal block-by-block
"""
import time
import odrive
import numpy as np
import lib
odrv0 = odrive.find_any()

# in another terminal, run:
# >>> python -m optoforce -f data/raw-opto-log.csv
# or just look at the reading on the scale.
# Then while that's running:
with lib.CurrentControl(odrv0):
    for i in [0.5, 1.0, 2.0, 3.0, 4.0, 5.0]:
        print(f'current = {i} [A]')
        lib.set_current(odrv0, motor0=i, motor1=0)
        time.sleep(1)

# then plot:
# >>> python scripts/plot_optoforce.py
# and read off data points

# the resulting data:
# input current, sensor reading (N)
data = np.array([
    [0., -91.1],
    [.5, -91.7],
    [1., -92.4],
    [2., -94.3],
    [3., -95.9],
    [4., -97.2],
    [5., -99.0],
])
# current, force and torque changes
di, df = data[-1, :] - data[1, :]
lever_length = lib.l1
dtau = df * lever_length
kt = - dtau / di
print(f'{di=:.3} A | {df=:.3} N | {dtau=:.3} Nm | {kt=:.3} Nm/A')

# => kt = 0.223 Nm

#%%
import time, odrive, numpy as np
import lib
odrv0 = odrive.find_any()
assert odrv0 is not None, "Couldn't find the odrive"

#%%
# in another cell, run:
# >>> python -m optoforce -f data/raw-opto-log.csv
# then while that's running:
with lib.CurrentControl(odrv0):
    for i in [0.5, 1.0, 2.0, 3.0, 4.0, 5.0]:
        print(f'current = {i} [A]')
        lib.set_current(odrv0, motor0=i, motor1=0)
        time.sleep(1)

# then plot:
# >>> python scripts/plot_optoforce.py
# and read off data points

#%% resulting data:
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
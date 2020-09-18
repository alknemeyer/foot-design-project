"""
Plots data logged by opto force to `data/raw-opto-log.csv`

Run as follows:

    $ python scripts/plot_optoforce.py

"""
import matplotlib.pyplot as plt
import numpy as np


def convert(s: bytes):
    "example s: 11:18:35:271027"
    hr, min, sec, ms = str(s, 'utf-8').split(':')
    return float(int(min)*60 + int(sec) + int(ms)/1000_000)


arr: np.ndarray = np.loadtxt(
    'data/raw-opto-log.csv', delimiter=',',
    skiprows=1, usecols=[0, 1, 2, 3],
    converters={0: convert},
)

tstart = arr[0, 0]
plt.plot(arr[:, 0] - tstart, arr[:, 1:])
plt.grid(True)
plt.title(f'OptoForce - logged data - {arr.shape[0]} data points')
plt.legend(('Fx', 'Fy', 'Fz'))
plt.xlabel('Time [$s$]')
plt.ylabel('Force [$N$]')
plt.show()

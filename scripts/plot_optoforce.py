"""
Plots data logged by opto force to `data/raw-opto-log.csv`

Run as follows:

    $ python scripts/plot_optoforce.py

"""
import matplotlib.pyplot as plt
import numpy as np
plt.style.use('seaborn')


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
t = arr[:, 0] - tstart
plt.plot(t, arr[:, 1:])
plt.grid(True)
plt.title(f'OptoForce - logged data - {arr.shape[0]} data points')
plt.legend(('Fx', 'Fy', 'Fz'))
plt.xlabel('Time [$s$]')
plt.ylabel('Force [$N$]')
plt.show()

###

# from matplotlib.animation import FuncAnimation
# fig, ax = plt.subplots()
# linex, = ax.plot([], [])
# liney, = ax.plot([], [])
# linez, = ax.plot([], [])
# def init():
#     plt.grid(True)
#     plt.title(f'Force data')
#     plt.legend(('Fx', 'Fy', 'Fz'))
#     plt.xlabel('Time [$s$]')
#     plt.ylabel('Force [$N$]')
#     ax.set_xlim(np.min(t), np.max(t))
#     ax.set_ylim(np.min(arr[:,1:]), np.max(arr[:,1:]))
#     return linex, liney, linez

# def update(frame: int):
#     linex.set_data(t[:frame], arr[:frame, 1])
#     liney.set_data(t[:frame], arr[:frame, 2])
#     linez.set_data(t[:frame], arr[:frame, 3])
#     return linex, liney, linez

# stride = 50
# nsamples = len(t)
# dt_ms = 1000*t[-1]/nsamples * stride
# ani = FuncAnimation(fig, update, init_func=init, interval=dt_ms, frames=range(0, nsamples, stride), blit=True)
# # plt.show()
# ani.save('data/force-animation.mp4')
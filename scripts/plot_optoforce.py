"""
Plots data logged by opto force to `data/raw-opto-log.csv`

Usage instructions:

    $ python scripts/plot_optoforce.py --help

"""
import matplotlib.pyplot as plt
import numpy as np
from argparse import ArgumentParser
plt.style.use('seaborn')

parser = ArgumentParser()
parser.add_argument('-a', '--animate', action='store_true',
                    help='show an animation. Otherwise, just plot')
parser.add_argument('--save', action='store_true',
                    help='if the animation should be saved to disk')
parser.add_argument('-i', '--inputfilename', default='data/raw-opto-log.csv',
                    help='.csv file of logged force data')
parser.add_argument('-f', '--filename', default='data/force-animation.mp4',
                    help='where the animation should be saved')
parser.add_argument('-s', '--start', type=int, default=0,
                    help='first index of the array')
parser.add_argument('-e', '--end', type=int, default=-1,
                    help='last index of the array')
args = parser.parse_args()


# def convert(s: bytes):
#     "example s: 11:18:35:271027"
#     hr, min, sec, ms = str(s, 'utf-8').split(':')
#     return float(int(min)*60 + int(sec) + int(ms)/1000_000)


arr: np.ndarray = np.loadtxt(  # type: ignore
    args.inputfilename, delimiter=',',
    skiprows=1, usecols=[0, 1, 2, 3],
    # converters={0: convert},
)

# arr = arr[12_000:28_000, :]
arr = arr[args.start:args.end]

tstart = arr[0, 0]
t = arr[:, 0] - tstart

if args.animate is False:
    plt.plot(t, arr[:, 1:])
    plt.grid(True)
    plt.title(f'OptoForce - logged data - {arr.shape[0]} data points')
    plt.legend(('Fx', 'Fy', 'Fz'))
    plt.xlabel('Time [$s$]')
    plt.ylabel('Force [$N$]')
    plt.show()

else:
    from matplotlib.animation import FuncAnimation
    fig = plt.figure(dpi=240)
    ax = plt.subplot()
    fig.set_size_inches(4/1.3, 5/1.3)
    plt.tight_layout(rect=(0.07, 0.02, 1, 0.98))
    linex, = ax.plot([], [])
    liney, = ax.plot([], [])
    linez, = ax.plot([], [])

    def init():
        plt.grid(True)
        plt.title('Hopping test')
        plt.legend(('$F_x$', '$F_y$', '$F_z$'), loc='lower right')
        plt.xlabel('Time [$s$]')
        plt.ylabel('Force [$N$]')
        ax.set_xlim(np.min(t), np.max(t))
        ax.set_ylim(np.min(arr[:, 1:]), np.max(arr[:, 1:]))
        return linex, liney, linez

    def update(frame: int):
        linex.set_data(t[:frame], arr[:frame, 1])
        liney.set_data(t[:frame], arr[:frame, 2])
        linez.set_data(t[:frame], arr[:frame, 3])
        return linex, liney, linez

    stride = 50
    nsamples = len(t)
    dt_ms = 1000*t[-1]/nsamples * stride
    ani = FuncAnimation(fig, update, init_func=init, interval=dt_ms,
                        frames=range(0, nsamples, stride), blit=True)

    if args.save:
        ani.save(args.filename)
    else:
        plt.show()

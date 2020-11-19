from time import time as t, sleep
from . import lib
from typing import TYPE_CHECKING, Union
if TYPE_CHECKING:
    from odrive import ODrive

ODRV: Union[None, 'ODrive'] = None


def get_odrv(): assert ODRV is not None; return ODRV
def foot_on_ground(): return True


class Stance:
    pausetime_s: float = 0.020

    def __init__(self) -> None:
        self.ts = t()

    def step(self):
        return Decompression() if t() - self.ts > self.pausetime_s else self


class Decompression:
    def __init__(self) -> None:
        lib.slow_gains(get_odrv())
        # set_foot_position(x_m=0, y_m=-0.010, vel=slow)####

    def step(self): return Flight() if foot_on_ground() else self


class Flight:
    def __init__(self) -> None:
        lib.slow_gains(get_odrv())
        # set_foot_position(x_m=0, y_m=-0.010, vel=slow)####

    def step(self): return Recovery() if foot_on_ground() else self


class Recovery:
    def step(self): return CompliantLanding() if True else self


class CompliantLanding:
    def step(self): return Stance() if True else self


class System:
    tloop_s: float = 0.010  # 100 Hz

    def __init__(self, odrv: 'ODrive') -> None:
        global ODRV
        ODRV = odrv

        self.state = Stance()

        with lib.PositionControl(odrv):
            while True:
                self.runloop()

    def runloop(self):
        ts = t()
        self.state = self.state.step()
        sleep(t() - ts)

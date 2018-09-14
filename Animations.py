from typing import Iterable, List

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
import matplotlib.artist as artist

from parameters import MassSpringDamper as msdParam
import signal_generator as sg


class Animation:
    """
    An abstract class for all animation classes
    """

    def __init__(self):
        self._do_init = True
        self.figure = plt.figure()
        self.axis = self.figure.add_subplot(111)


class MassSpringDamper(Animation):

    def __init__(self):
        Animation.__init__(self)
        self._width = msdParam.WIDTH
        self._height = msdParam.HEIGHT
        self.mass: patches.Rectangle = None

    def animate(self, z_0=0, z: Iterable=None) -> animation.FuncAnimation:

        def init_func() -> List[artist.Artist]:
            self.axis.set_xbound(-4, 4)
            self.axis.set_ybound(-3, 3)
            self.mass = patches.Rectangle((z_0, 0), 1, 1)
            self.axis.add_patch(self.mass)
            return [self.mass, ]

        def func(z_current):
            self.mass.xy = (z_current, 0)
            return [self.mass, ]

        return animation.FuncAnimation(self.figure, func=func, init_func=init_func, blit=True, frames=z)


class BallAndBeam(Animation):
    pass


class PlanarVTOL(Animation):
    pass


if __name__ == '__main__':
    a = MassSpringDamper()
    i = 0
    anim = a.animate(z=sg.sin())
    plt.waitforbuttonpress()
    plt.close()

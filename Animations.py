from typing import Iterable, List, Tuple

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
import matplotlib.artist as artist
import math

import parameters
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

    def __init__(self, **kwargs):
        super().__init__()
        params = {**kwargs, **parameters.MassSpringDamper}
        self._width = params['width']
        self._height = params['height']
        self.mass: patches.Rectangle = None

    def animate(self, z_0=0, z: Iterable=None) -> animation.FuncAnimation:

        def init_func() -> List[artist.Artist]:
            self.axis.set_xbound(-4, 4)
            self.axis.set_ybound(-3, 3)
            self.mass = patches.Rectangle((z_0, 0), self._width, self._height)
            self.axis.add_patch(self.mass)
            return [self.mass, ]

        def func(z_current):
            self.mass.xy = (z_current, 0)
            return [self.mass, ]

        return animation.FuncAnimation(self.figure, func=func, init_func=init_func, blit=True, frames=z)


class BallAndBeam(Animation):

    def __init__(self, **kwargs):
        super().__init__()
        params = {**kwargs, **parameters.BallAndBeam}
        self.beamLength = params['beamLength']
        self.beamWidth = params['beamWidth']
        self.ballRadius = params['ballRadius']
        self.beam: patches.Rectangle = None
        self.ball: patches.Circle = None

    def animate(self, q_0=(0, 0), q: Iterable=None) -> animation.FuncAnimation:

        def get_ball_xy(q_current: Tuple[float, float]) -> Tuple[float, float]:
            z = q_current[0]
            cos_a = math.cos(math.radians(q_current[1]))
            sin_a = math.sin(math.radians(q_current[1]))
            return z * cos_a - self.ballRadius * sin_a, z * sin_a + self.ballRadius * cos_a

        def init_func() -> List[artist.Artist]:
            bound = self.beamLength * 1.1
            self.axis.set_xbound(-bound, bound)
            self.axis.set_ybound(-bound, bound)

            xy_rectangle = (q_0[1], self.beamWidth / 2)
            self.beam = patches.Rectangle(xy_rectangle, self.beamLength, self.beamWidth, q_0[0])
            self.axis.add_patch(self.beam)

            self.ball = patches.Circle(get_ball_xy(q_0), self.ballRadius)
            self.axis.add_patch(self.ball)

            return [self.beam, self.ball, ]

        def func(q_current):
            self.beam.angle = q_current[1]
            xy = get_ball_xy(q_current)
            self.ball.center = xy
            return [self.beam, self.ball, ]

        return animation.FuncAnimation(self.figure, func=func, init_func=init_func, blit=True, frames=q, interval=40)


class PlanarVTOL(Animation):

    def __init__(self, **kwargs):
        super().__init__()
        params = {**kwargs, **parameters.PlanarVTOL}
        self.craftWidth = params['craftWidth']
        self.craftHeight = params['craftHeight']

    def animate(self, q_0=(0, 0, 0), q: Iterable = None) -> animation.FuncAnimation:

        def init_func() -> List[artist.Artist]:
            pass

        def func(q_curent):
            pass

        return animation.FuncAnimation(self.figure, func=func, init_func=init_func, blit=True, frames=q, interval=40)


if __name__ == '__main__':
    a = MassSpringDamper()
    i = 0
    # q_frames = zip(sg.sin(t_step=0.04, frequency=0.5, t_final=10), sg.sin(amplitude=180, t_step=0.04, frequency=0.7, t_final=10), )
    anim = a.animate(z=sg.sin(t_step=0.04, t_final=10))
    # movie_writer = animation.MovieWriter()
    anim.save("out/massSpringDamper.mp4", fps=20)
    plt.close()

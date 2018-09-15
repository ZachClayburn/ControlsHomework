from typing import Iterable, List, Tuple

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
import matplotlib.artist as artist
import math
import numpy as np

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
        """
        Animate a ball and beam system.

        :param q_0: The initial position of the system.In the form of (z, theta)' where z is the balls distance from
            the origin and theta is the beams angle, in radians.
        :param q: Iterable of the same format as q_o.
        :return: An animation object of the system.
        """

        def get_rotation(q_current: Tuple[float, float]) -> np.ndarray:
            cos_a = np.cos(q_current[1])
            sin_a = np.sin(q_current[1])
            return np.array([[cos_a, -sin_a], [sin_a, cos_a]])

        def init_func() -> List[artist.Artist]:
            bound = self.beamLength * 1.1
            self.axis.set_xbound(-bound, bound)
            self.axis.set_ybound(-bound, bound)

            rotation = get_rotation(q_0)

            xy_rectangle = rotation @ np.array([[0, -self.beamWidth / 2]]).T
            self.beam = patches.Rectangle(xy_rectangle, self.beamLength, self.beamWidth, q_0[0])
            self.axis.add_patch(self.beam)

            ball_xy = rotation @ np.array([[q_0[0], self.ballRadius + self.beamWidth / 2]]).T
            self.ball = patches.Circle(ball_xy, self.ballRadius, facecolor='red')
            self.axis.add_patch(self.ball)

            return [self.beam, self.ball, ]

        def func(q_current):
            rotation = get_rotation(q_current)
            self.beam.angle = math.degrees(q_current[1])
            self.beam.xy = rotation @ np.array([[0, -self.beamWidth / 2]]).T
            self.ball.center = rotation @ np.array([[q_current[0], self.ballRadius + self.beamWidth / 2]]).T
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


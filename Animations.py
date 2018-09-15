from typing import Iterable, List, Tuple
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
import matplotlib.artist as artist
import math
import numpy as np
import parameters


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
        self.bodyWidth = params['bodyWidth']
        self.bodyHeight = params['bodyHeight']
        self.wingSpacing = params['wingSpacing']
        self.wingMajorAxis = params['wingMajorAxis']
        self.wingMinorAxis = params['wingMinorAxis']

        self.body: patches.Rectangle = None
        self.leftWing: patches.Ellipse = None
        self.rightWing: patches.Ellipse = None
        self.target: patches.Rectangle = None

    def animate(self, q_0=(0, 0, 0, 0), q: Iterable = None) -> animation.FuncAnimation:
        """
        Animate the planar VTOL system.

        :param q_0: The initial position of the system, in the form (z_vehicle, z_target, height, theta)', where z is
            the horizontal position of the vehicle and target, respectively, height it the vertical elevation of the
            vehicle's C.O.M. and theta is the ccw rotation about the C.O.M. of the vehicle from vertical.
        :param q: Iterable of the same format as q_o.
        :return:  An animation object of the system.
        """

        def get_rotation(q_current: Tuple[float, float, float, float]) -> np.ndarray:
            cos_a = np.cos(q_current[3])
            sin_a = np.sin(q_current[3])
            return np.array([[cos_a, -sin_a], [sin_a, cos_a]])

        def init_func() -> List[artist.Artist]:
            self.axis.set_ybound(0, 4)
            self.axis.set_xbound(-2, 2)

            rotation = get_rotation(q_0)
            center_of_mass_xy = np.array([[q_0[0], q_0[2]]]).T

            body_xy = center_of_mass_xy + rotation @ np.array([[- self.bodyWidth / 2, - self.bodyHeight / 2]]).T
            self.body = patches.Rectangle(body_xy, self.bodyWidth, self.bodyHeight, math.degrees(q_0[3]))
            self.axis.add_patch(self.body)

            left_wing_xy = center_of_mass_xy + rotation @ np.array([[-self.wingSpacing, 0]]).T
            wing_params = {'width': self.wingMajorAxis, 'height': self.wingMinorAxis, 'angle': math.degrees(q_0[3])}
            self.leftWing = patches.Ellipse(left_wing_xy, **wing_params)
            self.axis.add_patch(self.leftWing)

            right_wing_xy = center_of_mass_xy + rotation @ np.array([[self.wingSpacing, 0]]).T
            wing_params = {'width': self.wingMajorAxis, 'height': self.wingMinorAxis, 'angle': math.degrees(q_0[3])}
            self.rightWing = patches.Ellipse(right_wing_xy, **wing_params)
            self.axis.add_patch(self.rightWing)

            self.target = patches.Rectangle((q_0[1] - self.bodyWidth / 2, 0), self.bodyWidth, self.bodyHeight)
            self.axis.add_patch(self.target)

            return [self.body, self.leftWing, self.rightWing, self.target, ]

        def func(q_current):
            rotation = get_rotation(q_current)
            center_of_mass_xy = np.array([[q_current[0], q_current[2]]]).T

            body_xy = center_of_mass_xy + rotation @ np.array([[- self.bodyWidth / 2, - self.bodyHeight / 2]]).T
            self.body.xy = body_xy
            self.body.angle = math.degrees(q_current[3])

            left_wing_xy = center_of_mass_xy + rotation @ np.array([[-self.wingSpacing, 0]]).T
            self.leftWing.center = left_wing_xy
            self.leftWing.angle = math.degrees(q_current[3])

            right_wing_xy = center_of_mass_xy + rotation @ np.array([[self.wingSpacing, 0]]).T
            self.rightWing.center = right_wing_xy
            self.rightWing.angle = math.degrees(q_current[3])

            self.target.xy = (q_current[1] - self.bodyWidth / 2, 0)

            return [self.body, self.leftWing, self.rightWing, self.target, ]

        return animation.FuncAnimation(self.figure, func=func, init_func=init_func, blit=True, frames=q, interval=40)


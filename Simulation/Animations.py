import math
from abc import ABC
from typing import Iterable, List, Tuple, Callable

import matplotlib.animation as animation
import matplotlib.artist as artist
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np

from Simulation import parameters


class Animation(ABC):
    """
    An abstract class for all animation classes
    """

    milliseconds_per_frame: int = 40

    def __init__(self):
        self._do_init = True
        self.figure = plt.figure()
        self.axis = self.figure.add_subplot(111)

    def get_init_func(self, q_0) -> Callable[[], List[artist.Artist]]:
        raise NotImplementedError

    def get_step_func(self) -> Callable[[Tuple], List[artist.Artist]]:
        raise NotImplementedError

    def animate_frame(self, q_current: Tuple):
        if self._do_init:
            self.get_init_func(q_current)()
            self._do_init = False
        else:
            self.get_step_func()(q_current)

    def animate(self, q_0, q: Iterable = None):
        return animation.FuncAnimation(self.figure, func=self.get_step_func(), init_func=self.get_init_func(q_0),
                                       blit=True, frames=q, interval=self.milliseconds_per_frame, repeat=False)


class MassSpringDamper(Animation):
    """
    In this system, q is z, where z is the displacement of the mass from equilibrium
    """

    def __init__(self):
        super().__init__()
        params = parameters.MassSpringDamper
        self._width = params.width
        self._height = params.height
        self.mass: patches.Rectangle = None

    def get_init_func(self, q_0: Tuple[float]) -> Callable[[], List[artist.Artist]]:
        def init_func() -> List[artist.Artist]:
            bound = self._height * 4
            self.axis.set_xbound((-bound + self._height) / 2, (bound + self._height) / 2)
            self.axis.set_ybound(0, bound)
            self.mass = patches.Rectangle((q_0[0], 0), self._width, self._height)
            self.axis.add_patch(self.mass)
            return [self.mass, ]
        return init_func

    def get_step_func(self) -> Callable[[Tuple[float]], List[artist.Artist]]:
        def func(q_current: Tuple[float]):
            self.mass.xy = (q_current[0], 0)
            return [self.mass, ]
        return func


class BallAndBeam(Animation):
    """
    In this system, q is in the form (z, z_r, theta)' where z is the balls distance from
    the origin, z_r is the requested z, and theta is the beams angle, in radians.
    """

    def __init__(self):
        super().__init__()
        params = parameters.BallAndBeam
        self.beamLength = params.beam_length
        self.beamWidth = params.beam_width
        self.ballRadius = params.ball_radius
        self.beam: patches.Rectangle = None
        self.ball: patches.Circle = None
        self.expected: patches.Circle = None

    def get_init_func(self, q_0: Tuple[float, float]) -> Callable[[], List[artist.Artist]]:
        def init_func() -> List[artist.Artist]:
            bound = self.beamLength * 1.1
            self.axis.set_xbound(-bound, bound)
            self.axis.set_ybound(-bound, bound)

            rotation = _get_rotation(q_0[2])

            xy_rectangle = rotation @ np.array([[0, -self.beamWidth / 2]]).T
            self.beam = patches.Rectangle(xy_rectangle, self.beamLength, self.beamWidth, q_0[0])
            self.axis.add_patch(self.beam)

            ball_xy = rotation @ np.array([[q_0[0], self.ballRadius + self.beamWidth / 2]]).T
            self.ball = patches.Circle(ball_xy, self.ballRadius, facecolor='red')
            self.axis.add_patch(self.ball)

            expected_xy = rotation @ np.array([[q_0[1], 0]]).T
            self.expected = patches.Circle(expected_xy, self.beamWidth, facecolor='red')
            self.axis.add_patch(self.expected)

            return [self.beam, self.ball, self.expected]
        return init_func

    def get_step_func(self) -> Callable[[Tuple[float, float]], List[artist.Artist]]:
        def func(q_current: Tuple[float, float]):
            rotation = _get_rotation(q_current[2])
            self.beam.angle = math.degrees(q_current[2])
            self.beam.xy = rotation @ np.array([[0, -self.beamWidth / 2]]).T
            self.ball.center = rotation @ np.array([[q_current[0], self.ballRadius + self.beamWidth / 2]]).T
            self.expected.center = rotation @ np.array([[q_current[1], 0]]).T
            return [self.beam, self.ball, self.expected]
        return func


class PlanarVTOL(Animation):
    """
    In this system, q is of the form (z_vehicle, z_target, height, theta)', where z is
    the horizontal position of the vehicle and target, respectively, height it the vertical elevation of the
    vehicle's C.O.M. and theta is the ccw rotation about the C.O.M. of the vehicle from vertical.
    """

    def __init__(self):
        super().__init__()
        params = parameters.PlanarVTOL
        self.bodyWidth = params.body_width
        self.bodyHeight = params.body_height
        self.wingSpacing = params.wing_spacing
        self.wingMajorAxis = params.wing_major_axis
        self.wingMinorAxis = params.wing_minor_axis

        self.body: patches.Rectangle = None
        self.leftWing: patches.Ellipse = None
        self.rightWing: patches.Ellipse = None
        self.target: patches.Rectangle = None

    def get_init_func(self, q_0: Tuple[float, float, float, float]) -> Callable[[], List[artist.Artist]]:
        def init_func() -> List[artist.Artist]:
            scale = 8
            horizontal_shift = 0
            self.axis.set_ybound(0, scale)
            self.axis.set_xbound(-scale / 2 + horizontal_shift, scale / 2 + horizontal_shift)

            rotation = _get_rotation(q_0[3])
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
        return init_func

    def get_step_func(self) -> Callable[[Tuple[float, float, float, float]], List[artist.Artist]]:
        def func(q_current: Tuple[float, float, float, float]):
            rotation = _get_rotation(q_current[3])
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
        return func


def _get_rotation(angle: float):
    cos_a = np.cos(angle)
    sin_a = np.sin(angle)
    return np.array([[cos_a, -sin_a], [sin_a, cos_a]])

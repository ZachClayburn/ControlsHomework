import itertools
from abc import ABC
from typing import Type, Iterable, List, Union

import numpy as np

from . import Animations, Dynamics, Controller


class Simulation(ABC):
    milliseconds_per_sim_step: int = 10
    seconds_per_sim_step: float = milliseconds_per_sim_step / 1000
    animation: Animations.Animation
    dynamics: Dynamics.Dynamics
    _controller: Controller.Controller

    def __init__(self, initial_position: List[float]):
        self.initial_positions = initial_position

    def view_animation(self, requests: Iterable[float]):
        return self.animation.animate(self.initial_positions, self._step_func(requests))

    def _step_func(self, requests: Iterable[Union[float, tuple]]) -> Iterable:
        for request, t_mill in \
                zip(requests, itertools.count(0, self.milliseconds_per_sim_step)):
            u = self._controller.callback(request)
            self.dynamics.propagate_dynamics(u)
            v = t_mill % self.animation.milliseconds_per_frame
            if v == self.animation.milliseconds_per_frame - self.milliseconds_per_sim_step:
                yield self.dynamics.animation_outputs(u)


class MassSpringDamper(Simulation):
    animation: Animations.MassSpringDamper
    dynamics: Dynamics.MassSpringDamper
    _controller: Controller.MassSpringDamper

    def __init__(self):
        self.animation = Animations.MassSpringDamper()
        self.dynamics = Dynamics.MassSpringDamper()
        self._controller: Controller.MassSpringDamper = None
        super().__init__(self.dynamics.animation_0)

    def add_controller(self, controller: Type[Controller.MassSpringDamper], *args, **kwargs):
        self._controller = controller(self, *args, **kwargs)


class BallAndBeam(Simulation):
    animation: Animations.BallAndBeam
    dynamics: Dynamics.BallAndBeam
    _controller: Controller.
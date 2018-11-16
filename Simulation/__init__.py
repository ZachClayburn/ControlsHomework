import itertools
from abc import ABC
from typing import Type, Iterable, List, Union, Tuple

import numpy as np
import matplotlib.animation as mpl_animation
import matplotlib.pyplot as plt
from matplotlib.artist import Artist

from . import Animations, Dynamics, Controller


class Simulation(ABC):
    milliseconds_per_sim_step: int = 10
    seconds_per_sim_step: float = milliseconds_per_sim_step / 1000
    animation: Animations.Animation
    dynamics: Dynamics.Dynamics
    _controller: Controller.Controller

    def __init__(self, initial_position: List[float]):
        self.initial_positions = initial_position

    def add_controller(self, controller: Type[Controller.Controller], *args, **kwargs):
        self._controller = controller(self, *args, **kwargs)

    def view_animation(self, requests: Iterable[Union[float, Tuple]]):
        return self.animation.animate(self.initial_positions, self._steps(requests))

    def save_movie(self, requests: Iterable[Union[float, Tuple]], save_file_name: str):
        fig = self.animation.figure
        ax = self.animation.axis

        def update_blit(artists_list: List[Artist]):
            fig.canvas.restore_region(bg_cache)
            for artist in artists_list:
                artist.axes.draw_artist(artist)

            ax.figure.canvas.blit(ax.bbox)

        artist_list = self.animation.get_init_func(self.initial_positions)()
        step_func = self.animation.get_step_func()

        for a in artist_list:
            a.set_animated(True)

        fig.canvas.draw()
        bg_cache = fig.canvas.copy_from_bbox(ax.bbox)

        # movie_writer = mpl_animation.MovieWriter(fps=25, )
        ffmpeg_writer = mpl_animation.writers['ffmpeg']
        movie_writer = ffmpeg_writer(fps=25)

        with movie_writer.saving(fig, save_file_name, 100):
            movie_writer.grab_frame()
            for frame in self._steps(requests):
                artist_list = step_func(frame)
                update_blit(artist_list)
                movie_writer.grab_frame()

    def view_plot(self, requests: Iterable[Union[float, Tuple]]):
        fig = self.animation.figure
        ax = self.animation.axis

        time = []
        state = []

        for request, t_mill in \
                zip(requests, itertools.count(0, self.milliseconds_per_sim_step)):
            time.append(t_mill / 1000)
            u = self._controller.callback(request)
            self.dynamics.propagate_dynamics(u)
            state.append(self.dynamics.z)

        ax.plot(time, state)

    def _steps(self, requests: Iterable[Union[float, tuple]]) -> Iterable:
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
        self._controller = None
        super().__init__(self.dynamics.animation_0)


class BallAndBeam(Simulation):
    animation: Animations.BallAndBeam
    dynamics: Dynamics.BallAndBeam
    _controller: Controller.BallAndBeam

    def __init__(self):
        self.animation = Animations.BallAndBeam()
        self.dynamics = Dynamics.BallAndBeam()
        self._controller = None
        super().__init__(self.dynamics.animation_0)


class PlanarVTOL(Simulation):
    animation: Animations.PlanarVTOL
    dynamics: Dynamics.PlanarVTOL
    _controller: Controller.PlanarVTOL

    def __init__(self):
        self.animation = Animations.PlanarVTOL()
        self.dynamics = Dynamics.PlanarVTOL()
        self._controller = None
        super().__init__(self.dynamics.animation_0)

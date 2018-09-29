from abc import ABC, abstractmethod
from typing import Iterable, List

import numpy as np


import parameters


class Dynamics(ABC):

    def __init__(self, state: np.ndarray):
        self.state: np.ndarray = state
        global_params = parameters.GlobalParameters
        self.sample_rate = global_params.sample_rate
        self.gravity = global_params.gravity

    def propagate_dynamics(self, u):

        # Integrate ODE using 4th order Runge-Kutta method
        k1 = self._derivatives(self.state, u)
        k2 = self._derivatives(self.state + self.sample_rate/2*k1, u)
        k3 = self._derivatives(self.state + self.sample_rate/2*k2, u)
        k4 = self._derivatives(self.state + self.sample_rate*k3, u)
        self.state += self.sample_rate/6 * (k1 + 2*k2 + 2*k3 + k4)

    @abstractmethod
    def _derivatives(self, state: np.ndarray, u) -> np.ndarray:
        raise NotImplementedError()

    @abstractmethod
    def outputs(self) -> np.ndarray:
        raise NotImplementedError()

    @abstractmethod
    def _animation_outputs(self)-> List:
        raise NotImplementedError()

    def states(self) -> np.ndarray:
        """
        Get the current states of the system
        :return: A numpy n-by-1 array where n is the number of state variables
        """
        return self.state

    def run_sim(self, inputs: Iterable) ->Iterable:
        for u in inputs:
            self.propagate_dynamics(u)
            yield self._animation_outputs()


class MassSpringDamper(Dynamics):
    """
    The dynamics simulation of the MassSpringDamper

    The state is in the form (z, zdot)T
    The animation output is in the form z
    """

    def __init__(self):
        params = parameters.MassSpringDamper
        self.mass = params.mass
        self.spring_const = params.spring_const
        self.damping = params.damping
        state = np.array([
            [params.z_0],
            [params.zdot_0]
        ])
        super().__init__(state=state)

    def _derivatives(self, state: np.ndarray, u) -> np.ndarray:
        A = np.array([
            [0, 1],
            [-self.spring_const / self.mass, -self.damping / self.mass]
        ])
        B = np.array([
            [0],
            [1 / self.mass]
        ])

        return A @ state + B * u

    def outputs(self) -> np.ndarray:
        pass

    def _animation_outputs(self) -> List:
        z = self.state.item(0)
        return [z]


class BallAndBeam(Dynamics):
    """
    The dynamics simulation for the BallAndBeam System

    The state is in the form (z, theta, zdot, thetaDot)T
    The animation output is in the form (z, theta)T
    """

    def __init__(self):
        params = parameters.BallAndBeam
        self.beam_length = params.beam_length
        self.beam_mass = params.beam_mass
        self.ball_mass = params.ball_mass
        state = np.array([
            [params.z_0],
            [params.theta_0],
            [params.zdot_0],
            [params.thetadot_0]
        ])
        super().__init__(state=state)

    def _derivatives(self, state: np.ndarray, u) -> np.ndarray:
        xdot = np.zeros((4, 1))

        z, theta, zdot, thetadot = state.T.tolist()[0]

        xdot[0:2] = state[2:4]
        xdot[2] = z * thetadot**2 - self.gravity * np.sin(theta)
        xdot[3] = u * self.beam_length * np.cos(theta) -\
                  2 * self.ball_mass * z * zdot * thetadot -\
                  self.ball_mass * self.gravity * z * np.cos(theta) -\
                  self.beam_mass * self.gravity * self.beam_length / 2 * np.cos(theta)
        xdot[3] /= self.beam_mass * self.beam_length**2 / 3 + self.ball_mass * z**2

        return xdot

    def outputs(self) -> np.ndarray:
        pass

    def _animation_outputs(self) -> List:
        return self.state[0:2].T.tolist()[0]


class PlanarVTOL(Dynamics):

    def __init__(self):
        params = parameters.BallAndBeam
        state = np.ndarray([])
        super().__init__(state=state)

    def _derivatives(self, state: np.ndarray, u: np.ndarray) -> np.ndarray:
        pass

    def outputs(self) -> np.ndarray:
        pass


if __name__ == '__main__':
    msd = MassSpringDamper()
    msd.propagate_dynamics(1)

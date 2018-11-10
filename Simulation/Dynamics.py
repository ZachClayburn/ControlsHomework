from abc import ABC, abstractmethod
from typing import Iterable, List
import time

import numpy as np

from Simulation import parameters


class Dynamics(ABC, parameters.GlobalParameters):

    def __init__(self, state: np.ndarray):
        self.state: np.ndarray = state

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
    def animation_outputs(self, u)-> List:
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
            yield self.animation_outputs(u)


class MassSpringDamper(Dynamics, parameters.MassSpringDamper):
    """
    The dynamics simulation of the MassSpringDamper

    The state is in the form (z, zdot)T
    The animation output is in the form z
    """
    def __init__(self):
        super().__init__(state=np.asarray(self.state_0))
        self.A_real = np.array([
            [0, 1],
            [-self.spring_const_real / self.mass_real, -self.damping_real / self.mass_real]
        ])
        self.B_real = np.array([
            [0],
            [1 / self.mass_real]
        ])

    def _derivatives(self, state: np.ndarray, u) -> np.ndarray:
        return self.A_real @ state + self.B_real * u

    def outputs(self) -> np.ndarray:
        pass

    def animation_outputs(self, u) -> List:
        return [self.z]

    def __getattr__(self, item):
        return {
            'z': self.state.item(0),
            'zdot': self.state.item(1),
        }[item]


class BallAndBeam(Dynamics, parameters.BallAndBeam):
    """
    The dynamics simulation for the BallAndBeam System

    The state is in the form (z, theta, zdot, thetaDot)T
    The animation output is in the form (z, theta)T
    The input is in the form (force, z_r)
    """

    def __init__(self):
        super().__init__(state=np.asanyarray(self.state_0))

    def _derivatives(self, state: np.ndarray, u) -> np.ndarray:
        xdot = np.zeros((4, 1))

        z, theta, zdot, thetadot = state.T.tolist()[0]
        force, z_r = u.tolist()

        xdot[0:2] = state[2:4]
        xdot[2] = z * thetadot**2 - self.gravity * np.sin(theta)
        xdot[3] = force * self.beam_length_real * np.cos(theta) -\
                  2 * self.ball_mass_real * z * zdot * thetadot -\
                  self.ball_mass_real * self.gravity * z * np.cos(theta) -\
                  self.beam_mass_real * self.gravity * self.beam_length_real / 2 * np.cos(theta)
        xdot[3] /= self.beam_mass_real * self.beam_length_real**2 / 3 + self.ball_mass_real * z**2

        return xdot

    def outputs(self) -> np.ndarray:
        pass

    def animation_outputs(self, u: np.ndarray) -> List:
        return [self.z, u.item(1), self.theta]

    def __getattr__(self, item):
        return {
            'z': self.state.item(0),
            'theta': self.state.item(1),
            'zdot': self.state.item(2),
            'thetadot': self.state.item(3),
        }[item]


class PlanarVTOL(Dynamics, parameters.PlanarVTOL):
    """
    The dynamics simulation for the PlanarVtol System

    The state is in the form (z, height, theta, zdot, heightdot, thetadot)T
    The animation output is in the form (z_vehicle, z_target, height, theta)T
    The input is in the form (force_left, force_right, z_target)
    """

    def __init__(self):
        super().__init__(state=np.asarray(self.state_0))

    def _derivatives(self, state: np.ndarray, u) -> np.ndarray:
        xdot = np.zeros((6, 1))
        xdot[0:3] = state[3:6]

        z, height, theta, zdot, heightdot, thetadot = state.T.tolist()[0]
        force_left, force_right, z_target = u

        combined_mass = self.center_mass_real + 2 * self.wing_mass
        combined_moi = self.center_moi_real + 2 * self.wing_mass * self.wing_spacing_real**2

        a_matrix = np.zeros((3, 3))
        a_matrix[0, 0] = combined_mass
        a_matrix[1, 1] = combined_mass
        a_matrix[2, 2] = combined_moi

        b_vector = np.array([
            [-(force_right + force_left) * np.sin(theta) - self.drag_real * zdot],
            [-combined_mass * self.gravity + (force_right + force_left) * np.cos(theta)],
            [self.wing_spacing_real * (force_right - force_left)],
        ])
        xdot[3:6] = np.linalg.solve(a_matrix, b_vector)
        return xdot

    def outputs(self) -> np.ndarray:
        pass

    def animation_outputs(self, u) -> List:
        return [self.state.item(0), u[2], self.state.item(1), self.state.item(2)]

    def __getattr__(self, item):
        return {
            'z': self.state.item(0),
            'h': self.state.item(1),
            'theta': self.state.item(2),
            'zdot': self.state.item(3),
            'hdot': self.state.item(4),
            'thetadot': self.state.item(5),
        }[item]


if __name__ == '__main__':
    msd = MassSpringDamper()
    msd.propagate_dynamics(1)

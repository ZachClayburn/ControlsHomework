from typing import Union

import Simulation
import numpy as np


class MassSpringDamper(Simulation.Controller.MassSpringDamper):

    def __init__(self, system: Simulation.MassSpringDamper,
                 feedback_gains: np.ndarray, reference_gain: np.ndarray,
                 max_force: float = np.inf):
        super().__init__()
        self.dynamics = system.dynamics
        self.feedback_gains = feedback_gains
        self.reference_gain = reference_gain
        self.max_force = max_force

        self.prev_z: float = 0.0

    def callback(self, request: Union[float, tuple]) -> np.ndarray:
        dt = self.dynamics.sample_rate

        z = self.dynamics.z
        z_dot = (z - self.prev_z) / dt

        self.prev_z = z

        state = np.asarray([
            [z],
            [z_dot]
        ])

        z_r = np.asarray(request)
        force = - self.feedback_gains.dot(state) + self.reference_gain * z_r

        if force > self.max_force:
            force[0] = self.max_force

        return force


class BallAndBeam(Simulation.Controller.BallAndBeam):

    def __init__(self, system: Simulation.MassSpringDamper,
                 feedback_gains: np.ndarray, reference_gain: np.ndarray,
                 max_force: float = np.inf):
        super().__init__()
        self.dynamics = system.dynamics
        self.feedback_gains = feedback_gains
        self.reference_gain = reference_gain
        self.max_force = max_force

        self.prev_z: float = self.dynamics.z
        self.prev_theta: float = self.dynamics.theta

    def callback(self, request: Union[float, tuple]) -> np.ndarray:
        dt = self.dynamics.sample_rate

        z = self.dynamics.z
        z_dot = (z - self.prev_z) / dt
        theta = self.dynamics.theta
        theta_dot = (theta - self.prev_theta) / dt

        self.prev_z = z
        self.prev_theta = theta

        state = np.asarray([
            [z],
            [theta],
            [z_dot],
            [theta_dot],
        ])

        z_r = np.asarray(request)
        z_r.shape = (1, 1)
        u = - self.feedback_gains.dot(state) + self.reference_gain * z_r

        equilibrium_force = \
            self.dynamics.gravity * \
            (self.dynamics.ball_mass * self.dynamics.z /
             self.dynamics.beam_length + self.dynamics.beam_mass / 2)

        u[0] += equilibrium_force

        if u > self.max_force:
            u[0] = self.max_force

        return np.concatenate((u, z_r)).T[0]


class PlanarVTOL(Simulation.Controller.PlanarVTOL):

    def __init__(self, system: Simulation.MassSpringDamper,
                 feedback_gains_lat: np.ndarray, reference_gain_lat: np.ndarray,
                 feedback_gains_lon: np.ndarray, reference_gain_lon: np.ndarray,
                 max_force: float = np.inf):
        super().__init__()
        self.dynamics = system.dynamics
        self.feedback_gains_lat = feedback_gains_lat
        self.reference_gain_lat = reference_gain_lat
        self.feedback_gains_lon = feedback_gains_lon
        self.reference_gain_lon = reference_gain_lon
        self.max_force = max_force

        self.prev_h: float = self.dynamics.h
        self.prev_z: float = self.dynamics.z
        self.prev_theta: float = self.dynamics.theta

    def callback(self, request: Union[float, tuple]) -> np.ndarray:
        h_request, z_request = request

        force = self._h_callback(h_request)
        torque = self._z_callback(z_request)

        equilibrium_force = (self.dynamics.center_mass + 2 * self.dynamics.wing_mass) * self.dynamics.gravity

        force += equilibrium_force

        d = self.dynamics.wing_spacing
        transform = np.array([
            [1, 1],
            [-d, d],
        ])

        x = np.array([
            [force],
            [torque],
        ])

        left_force, right_force = np.linalg.solve(transform, x).T.tolist()[0]

        if left_force > self.max_force:
            print(f'Saturation: left_force = {left_force}')
            left_force = self.max_force
        if left_force < 0:
            print(f'Saturation: left_force = {left_force}')
            left_force = 0

        if right_force > self.max_force:
            print(f'Saturation: right_force = {right_force}')
            right_force = self.max_force
        if right_force < 0:
            print(f'Saturation: right_force = {right_force}')
            right_force = 0

        u = np.array([left_force, right_force, z_request])
        return u

    def _z_callback(self, z_request: float):
        dt = self.dynamics.sample_rate

        z = self.dynamics.z
        z_dot = (z - self.prev_z) / dt
        theta = self.dynamics.theta
        theta_dot = (theta - self.prev_theta) / dt

        self.prev_z = z
        self.prev_theta = theta

        state = np.asarray([
            [z],
            [theta],
            [z_dot],
            [theta_dot],
        ])

        torque = - self.feedback_gains_lat @ state + self.reference_gain_lat * z_request

        return torque.item(0)

    def _h_callback(self, h_request: float):
        dt = self.dynamics.sample_rate

        h = self.dynamics.h
        h_dot = (h - self.prev_h) / dt

        self.prev_h = h

        state = np.asarray([
            [h],
            [h_dot],
        ])

        force = - self.feedback_gains_lon @ state + self.reference_gain_lon * h_request
        return force.item(0)


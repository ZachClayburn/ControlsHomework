from abc import ABC
from typing import Union, Tuple

import numpy as np

import Simulation


class MassSpringDamper(Simulation.Controller.MassSpringDamper):
    def __init__(self, system: Simulation.MassSpringDamper,
                 proportional_gain: float, derivative_gain: float,
                 max_force: float = np.inf):
        super().__init__()
        self.dynamics = system.dynamics
        self.proportional_gain = proportional_gain
        self.derivative_gain = derivative_gain
        self.max_force = max_force

    def callback(self, z_requested: float) -> np.ndarray:
        spring_const = self.dynamics.spring_const

        z = self.dynamics.z
        zdot = self.dynamics.zdot
        # equilibrium_force can be set to a non-zero value to remove steady state error
        equilibrium_force = spring_const * 0

        force_tilde = self.proportional_gain * (z_requested - z) - self.derivative_gain * zdot
        force = force_tilde + equilibrium_force

        # Saturation
        if force > self.max_force:
            print(f'Saturation: {force}')
            force = self.max_force
        if force < -self.max_force:
            print(f'Saturation: {force}')
            force = -self.max_force

        u = np.array([force])
        return u


class BallAndBeam(Simulation.Controller.BallAndBeam):
    def __init__(self, system: Simulation.BallAndBeam,
                 proportional_gain_z: float, derivative_gain_z: float,
                 proportional_gain_theta: float, derivative_gain_theta: float,
                 max_force: float = np.inf):
        super().__init__()
        self.dynamics = system.dynamics
        self.proportional_gain_z = proportional_gain_z
        self.derivative_gain_z = derivative_gain_z
        self.proportional_gain_theta = proportional_gain_theta
        self.derivative_gain_theta = derivative_gain_theta
        self.max_force = max_force

    def callback(self, request: Union[float, tuple]) -> np.ndarray:
        theta_request = self._z_callback(request)
        return self._theta_callback(theta_request)

    def _z_callback(self, z_request: float):
        error = z_request - self.dynamics.z
        theta_request = self.proportional_gain_z * error - self.dynamics.zdot * self.derivative_gain_z

        return theta_request

    def _theta_callback(self, theta_request: float):
        error = theta_request - self.dynamics.theta
        force_tilde = self.proportional_gain_theta * error - self.dynamics.thetadot * self.derivative_gain_theta
        print(force_tilde)
        z_equilibrium = self.dynamics.z
        ball_mass = self.dynamics.ball_mass
        beam_mass = self.dynamics.beam_mass
        gravity = self.dynamics.gravity
        beam_length = self.dynamics.beam_length
        equilibrium_force = (ball_mass * z_equilibrium / beam_length + beam_mass / 2) * gravity

        force = force_tilde + equilibrium_force

        if force > self.max_force:
            print(f'Saturation: {force}')
            force = self.max_force
        if force < -self.max_force:
            print(f'Saturation: {force}')
            force = -self.max_force

        u = np.array([force])
        return u


class PlanarVTOL(Simulation.Controller.BallAndBeam):
    def __init__(self, system: Simulation.PlanarVTOL,
                 proportional_gain_h: float, derivative_gain_h: float,
                 proportional_gain_z: float, derivative_gain_z: float,
                 proportional_gain_theta: float, derivative_gain_theta: float,
                 max_force: float = np.inf):
        super().__init__()
        self.dynamics = system.dynamics
        self.proportional_gain_h = proportional_gain_h
        self.derivative_gain_h = derivative_gain_h
        self.proportional_gain_z = proportional_gain_z
        self.derivative_gain_z = derivative_gain_z
        self.proportional_gain_theta = proportional_gain_theta
        self.derivative_gain_theta = derivative_gain_theta
        self.max_force = max_force

    def callback(self, requested: Tuple[float, float]) -> np.ndarray:
        """
        Callback for the PlanarVTOL PD controller
        :param requested: The requested position in the form (h_r, z_r)
        :return:
        """
        h_r, z_r = requested
        h_error = h_r - self.dynamics.h

        center_mass = self.dynamics.center_mass
        wing_mass = self.dynamics.wing_mass
        gravity = self.dynamics.gravity
        equilibrium_force = (center_mass + 2 * wing_mass) * gravity

        force_tilde = self.proportional_gain_h * h_error - self.derivative_gain_h * self.dynamics.hdot

        force = force_tilde + equilibrium_force

        theta_r = self._z_callback(z_r)
        torque = self._theta_callback(theta_r)

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
        if left_force < -self.max_force:
            print(f'Saturation: left_force = {left_force}')
            left_force = -self.max_force

        if right_force > self.max_force:
            print(f'Saturation: right_force = {right_force}')
            right_force = self.max_force
        if right_force < -self.max_force:
            print(f'Saturation: right_force = {right_force}')
            right_force = -self.max_force

        u = np.array([left_force, right_force, z_r])
        return u

    def _z_callback(self, z_request: float) -> float:
        error = z_request - self.dynamics.z
        theta_request = self.proportional_gain_z * error - self.dynamics.zdot * self.derivative_gain_z

        return theta_request

    def _theta_callback(self, theta_request: float) -> float:
        error = theta_request - self.dynamics.theta
        torque = self.proportional_gain_theta * error - self.dynamics.thetadot * self.derivative_gain_theta

        return torque

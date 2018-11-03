from typing import Union

import numpy as np

import Simulation


class MassSpringDamper(Simulation.Controller.MassSpringDamper):

    def __init__(self, system: Simulation.MassSpringDamper,
                 proportional_gain: float, derivative_gain: float, integrator_gain: float,
                 max_force: float = np.inf):
        super().__init__()
        self.dynamics = system.dynamics
        self.proportional_gain = proportional_gain
        self.derivative_gain = derivative_gain
        self.integrator_gain = integrator_gain
        self.max_force = max_force

        self.prev_z: float = 0.0
        self.prev_error: float = 0.0
        self.int: float = 0.0

    def callback(self, request: float) -> np.ndarray:
        dt = self.dynamics.sample_rate

        z = self.dynamics.z
        z_dot = (z - self.prev_z) / dt
        z_error = request - z

        self.prev_z = z

        self.int += (dt / 2) * (z_error + self.prev_error)
        force = self.proportional_gain * z_error + self.integrator_gain * self.int - self.derivative_gain * z_dot

        self.prev_error = z_error

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
                 proportional_gain_z: float, derivative_gain_z: float, integrator_gain_z: float,
                 proportional_gain_theta: float, derivative_gain_theta: float, integrator_gain_theta: float,
                 thresh_z: float = np.inf, thresh_theta: float = np.inf, max_force: float = np.inf):
        super().__init__()
        self.dynamics = system.dynamics
        self.proportional_gain_z = proportional_gain_z
        self.derivative_gain_z = derivative_gain_z
        self.integrator_gain_z = integrator_gain_z
        self.proportional_gain_theta = proportional_gain_theta
        self.derivative_gain_theta = derivative_gain_theta
        self.integrator_gain_theta = integrator_gain_theta
        self.max_force = max_force
        self.thresh_theta = thresh_theta
        self.thresh_z = thresh_z

        self.prev_z: float = 0.0
        self.prev_error_z: float = 0.0
        self.int_z: float = 0.0

        self.prev_theta: float = 0.0
        self.prev_error_theta: float = 0.0
        self.int_theta: float = 0.0

    def callback(self, request: float) -> np.ndarray:
        theta_request = self._z_callback(request)
        force = self._theta_callback(theta_request)

        equilibrium_force = \
            self.dynamics.gravity * \
            (self.dynamics.ball_mass * self.dynamics.z /
             self.dynamics.beam_length + self.dynamics.beam_mass / 2)

        force += equilibrium_force

        if force > self.max_force:
            print(f'Saturation: {force}')
            force = self.max_force
        if force < -self.max_force:
            print(f'Saturation: {force}')
            force = -self.max_force

        # print(f"Z Requested:{request} Force:{force}")

        u = np.array([force])
        return u

    def _z_callback(self, z_request: float) -> float:
        dt = self.dynamics.sample_rate

        z = self.dynamics.z
        error = z_request - z
        z_dot = (z - self.prev_z) / dt
        print(f"measured:{self.dynamics.zdot} calculated:{z_dot}")
        error_dot = (error - self.prev_error_z) / dt

        if abs(error_dot) < self.thresh_z:
            self.int_z += (dt / 2) * (error + self.prev_error_z)

        self.prev_z = z
        self.prev_error_z = error

        return \
            self.proportional_gain_z * error + \
            self.integrator_gain_z * self.int_z - \
            self.derivative_gain_z * z_dot

    def _theta_callback(self, theta_request: float) -> float:
        dt = self.dynamics.sample_rate

        theta = self.dynamics.theta
        error = theta_request - theta
        theta_dot = (theta - self.prev_theta) / dt
        error_dot = (error - self.prev_error_theta) / dt

        if abs(error_dot) < self.thresh_theta:
            self.int_theta += (dt / 2) * (error + self.prev_error_theta)

        self.prev_theta = theta
        self.prev_error_theta = error

        return \
            self.proportional_gain_theta * error + \
            self.integrator_gain_theta * self.int_z - \
            self.proportional_gain_theta * theta_dot


class PlanarVTOL(Simulation.Controller.PlanarVTOL):

    def __init__(self, system: Simulation.PlanarVTOL,
                 proportional_gain_h: float, derivative_gain_h: float, integrator_gain_h: float,
                 proportional_gain_z: float, derivative_gain_z: float, integrator_gain_z: float,
                 proportional_gain_theta: float, derivative_gain_theta: float, integrator_gain_theta: float,
                 max_force: float = np.inf):
        super().__init__()
        self.dynamics = system.dynamics
        self.proportional_gain_h = proportional_gain_h
        self.derivative_gain_h = derivative_gain_h
        self.integrator_gain_h = integrator_gain_h
        self.proportional_gain_z = proportional_gain_z
        self.derivative_gain_z = derivative_gain_z
        self.integrator_gain_z = integrator_gain_z
        self.proportional_gain_theta = proportional_gain_theta
        self.derivative_gain_theta = derivative_gain_theta
        self.integrator_gain_theta = integrator_gain_theta
        self.max_force = max_force

    def callback(self, request: Union[float, tuple]) -> np.ndarray:
        pass

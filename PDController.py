import numpy as np
import Simulation.Controller as Controller
from Simulation import MassSpringDamper as MSDSim


class MassSpringDamper(Controller.MassSpringDamper):
    def __init__(self, system: MSDSim, proportional_gain: float, derivative_gain: float, max_force: float = np.inf):
        self.dynamics = system.dynamics
        self.proportional_gain = proportional_gain
        self.derivative_gain = derivative_gain
        self.max_force = max_force;

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

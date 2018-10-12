import numpy as np
import Simulation.Controller as Controller
from Simulation import MassSpringDamper as MSDSim


class MassSpringDamper(Controller.MassSpringDamper):
    def __init__(self, system: MSDSim, proportional_gain: float, derivative_gain: float):
        self.dynamics = system.dynamics
        self.proportional_gain = proportional_gain
        self.derivative_gain = derivative_gain

    def callback(self, requests: float) -> np.ndarray:
        spring_const = self.dynamics.spring_const

        z = self.dynamics.z
        z_requested = requests
        equilibrium_force = spring_const * z

        force_tilde = self.proportional_gain * (z_requested - z) - self.derivative_gain * z
        force = force_tilde + equilibrium_force

        u = np.array([force])
        return u

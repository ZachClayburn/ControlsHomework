import numpy as np

import Simulation


class MassSpringDamper(Simulation.Controller.MassSpringDamper):

    def __init__(self, system: Simulation.MassSpringDamper,
                 proportional_gain: float, derivative_gain: float, integrator_gain: float,
                 max_force: float=np.inf):
        super().__init__()
        self.dynamics = system.dynamics
        self.proportional_gain = proportional_gain
        self.derivative_gain = derivative_gain
        self.integrator_gain = integrator_gain
        self.max_force = max_force


class BallAndBeam(Simulation.Controller.BallAndBeam):

    def __init__(self, system: Simulation.BallAndBeam,
                 proportional_gain_z: float, derivative_gain_z: float, integrator_gain_z: float,
                 proportional_gain_theta: float, derivative_gain_theta: float, integrator_gain_theta: float,
                 max_force: float=np.inf):
        super().__init__()
        self.dynamics = system.dynamics
        self.proportional_gain_z = proportional_gain_z
        self.derivative_gain_z = derivative_gain_z
        self.integrator_gain_z = integrator_gain_z
        self.proportional_gain_theta = proportional_gain_theta
        self.derivative_gain_theta = derivative_gain_theta
        self.integrator_gain_theta = integrator_gain_theta
        self.max_force = max_force


class PlanarVTOL(Simulation.Controller.PlanarVTOL):

    def __init__(self, system: Simulation.PlanarVTOL,
                 proportional_gain_h: float, derivative_gain_h: float, integrator_gain_h: float,
                 proportional_gain_z: float, derivative_gain_z: float, integrator_gain_z: float,
                 proportional_gain_theta: float, derivative_gain_theta: float, integrator_gain_theta: float,
                 max_force: float=np.inf):
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

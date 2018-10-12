from . import Animations, Dynamics
from abc import ABC


class Simulation(ABC):
    pass


class MassSpringDamper(Simulation):
    def __init__(self):
        self.dynamics = Dynamics.MassSpringDamper
        self.animation = Animations.MassSpringDamper

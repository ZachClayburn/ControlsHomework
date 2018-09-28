from abc import ABC, abstractmethod

import numpy as np


import parameters


class Dynamics(ABC):

    def __init__(self, state: np.ndarray):
        self.state: np.ndarray = state

    @abstractmethod
    def propagate_dynamics(self) -> np.ndarray:
        raise NotImplementedError()

    @abstractmethod
    def derivatives(self, u: np.ndarray) -> np.ndarray:
        raise NotImplementedError()

    @abstractmethod
    def outputs(self) -> np.ndarray:
        raise NotImplementedError()

    def states(self) -> np.ndarray:
        """
        Get the current states of the system
        :return: A numpy n-by-1 array where n is the number of state variables
        """
        return self.state


class MassSpringDamper(Dynamics):

    def __init__(self):
        params = parameters.MassSpringDamper
        state = np.array([])
        super().__init__(state=state)

    def propagate_dynamics(self) -> np.ndarray:
        pass

    def derivatives(self, u: np.ndarray) -> np.ndarray:
        pass

    def outputs(self) -> np.ndarray:
        pass


class BallAndBeam(Dynamics):

    def __init__(self):
        params = parameters.BallAndBeam
        state = np.ndarray([])
        super().__init__(state=state)

    def propagate_dynamics(self) -> np.ndarray:
        pass

    def derivatives(self, u: np.ndarray) -> np.ndarray:
        pass

    def outputs(self) -> np.ndarray:
        pass


class PlanarVTOL(Dynamics):

    def __init__(self):
        params = parameters.BallAndBeam
        state = np.ndarray([])
        super().__init__(state=state)

    def propagate_dynamics(self) -> np.ndarray:
        pass

    def derivatives(self, u: np.ndarray) -> np.ndarray:
        pass

    def outputs(self) -> np.ndarray:
        pass

from typing import Iterable, Union

import numpy as np
from abc import ABC


class Controller(ABC):

    def __init__(self, *args, **kwargs):
        pass

    def callback(self, request: Union[float, tuple]) -> np.ndarray:
        raise NotImplementedError


class MassSpringDamper(Controller, ABC):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class BallAndBeam(Controller, ABC):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class PlanarVTOL(Controller, ABC):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

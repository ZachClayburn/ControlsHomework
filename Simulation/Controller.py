from typing import Iterable, Union

import numpy as np
from abc import ABC


class Controller(ABC):

    def callback(self, request: Union[float, tuple]) -> np.ndarray:
        raise NotImplementedError


class MassSpringDamper(Controller, ABC):
    def __init__(self, *args, **kwargs):
        raise NotImplementedError

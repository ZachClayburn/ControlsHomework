import math
import numpy as np
from typing import Iterable, Callable


def generator(signal_fun: Callable[[float, float, float, float], float],
              frequency: float = 1, amplitude: float = 1, y_offset: float = 0,
              t_start: float = 0, t_step: float = 0.04, t_final: float = math.inf) -> Iterable[float]:
    time = t_start
    while time <= t_final:
        yield signal_fun(time, frequency, amplitude, y_offset)
        time += t_step


def continuous(signal_fun: Callable[[float, float, float, float], float],
               frequency: float = 1, amplitude: float = 1, y_offset: float = 0) -> Callable[[float], float]:
    def out_fun(time: float) -> float:
        return signal_fun(time, frequency, amplitude, y_offset)

    return out_fun


def square(time: float, frequency: float = 1, amplitude: float = 1, y_offset: float = 0) -> float:
    if time % (1.0 / frequency) <= 0.5 / frequency:
        return y_offset + amplitude
    else:
        return y_offset - amplitude


def sawtooth(time: float, frequency: float = 1, amplitude: float = 1, y_offset: float = 0) -> float:
    return 4 * amplitude * frequency * (time % 1.0 / frequency) - amplitude + y_offset


def random(time: float, frequency: float = 1, amplitude: float = 1, y_offset: float = 0) -> float:
    return np.sqrt(amplitude) * np.random.rand() + y_offset


def sin(time: float, frequency: float = 1, amplitude: float = 1, y_offset: float = 0) -> float:
    return amplitude * np.sin(2 * np.pi * frequency * time) + y_offset


def cos(time: float, frequency: float = 1, amplitude: float = 1, y_offset: float = 0) -> float:
    return amplitude * np.cos(2 * np.pi * frequency * time) + y_offset


def constant(time: float, frequency: float = 1, amplitude: float = 1, y_offset: float = 0) -> float:
    return amplitude + y_offset


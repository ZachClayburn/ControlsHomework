import math
import numpy as np
from typing import Iterable


def square(frequency: float=1, amplitude: float=1, y_offset: float=0,
           t_start: float=0, t_step: float=0.1, t_final: float=math.inf) -> Iterable[float]:
    time = t_start
    while time < t_final:
        if time % (1.0/frequency) <= 0.5/frequency:
            yield y_offset + amplitude
        else:
            yield y_offset - amplitude
        time += t_step


def sawtooth(frequency: float=1, amplitude: float=1, y_offset: float=0,
             t_start: float=0, t_step: float=0.1, t_final: float=math.inf) -> Iterable[float]:
    time = t_start
    while time <= t_final:
        yield 4 * amplitude * frequency * (time % 1.0 / frequency) - amplitude + y_offset
        time += t_step


def random(frequency: float=1, amplitude: float=1, y_offset: float=0,
           t_start: float=0, t_step: float=0.1, t_final: float=math.inf) -> Iterable[float]:
    time = t_start
    while time <= t_final:
        yield np.sqrt(amplitude) * np.random.rand() + y_offset
        time += t_step


def sin(frequency: float=1, amplitude: float=1, y_offset: float=0,
        t_start: float=0, t_step: float=0.1, t_final: float=math.inf) -> Iterable[float]:
    time = t_start
    while time <= t_final:
        yield amplitude * np.sin(2 * np.pi * frequency * time) + y_offset
        time += t_step


def constant(frequency: float=1, amplitude: float=1, y_offset: float=0,
             t_start: float=0, t_step: float=0.1, t_final: float=math.inf) -> Iterable[float]:
    time = t_start
    while time <= t_final:
        yield y_offset
        time += t_step


if __name__ == '__main__':
    for t in random(t_final=5, frequency=5):
        print(t)

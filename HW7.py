import numpy as np

import PIDController as PID
import Simulation as Sim
import Simulation.signal_generator as sg
import Simulation.parameters as params


def d_10_c():
    params.randomness_parameter = 0
    msd = Sim.MassSpringDamper()
    max_force = 2.0
    damping_ratio = 1 / (2 ** (1 / 2))

    proportional_gain = max_force
    natural_frequency = ((msd.dynamics.spring_const + proportional_gain) / msd.dynamics.mass) ** (1 / 2)
    derivative_gain = msd.dynamics.mass * 2 * damping_ratio * natural_frequency - msd.dynamics.damping

    integrator_gain = 1

    msd.add_controller(PID.MassSpringDamper, proportional_gain, derivative_gain, integrator_gain, max_force)

    request = sg.generator(sg.constant, amplitude=2/3, t_step=msd.seconds_per_sim_step, t_final=30)

    handle = msd.view_animation(request)
    Sim.Animations.plt.waitforbuttonpress()
    Sim.Animations.plt.close()
    return handle


if __name__ == '__main__':
    d_10_c()

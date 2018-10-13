import numpy as np

import PDController as PD
import Simulation as Sim
import Simulation.signal_generator as sg


def d_7_d():
    msd = Sim.MassSpringDamper()
    proportional_gain = 4.5
    derivative_gain = 12
    msd.add_controller(PD.MassSpringDamper, proportional_gain, derivative_gain)
    request = sg.generator(sg.constant, t_step=msd.seconds_per_sim_step, t_final=5)
    handle = msd.view_animation(request)
    Sim.Animations.plt.waitforbuttonpress()
    Sim.Animations.plt.close()


def d_8_a():
    msd = Sim.MassSpringDamper()
    damping_ratio = 1 / 2**(1/2)
    rise_time = 2
    natural_frequency = np.pi / (2 * rise_time * (1 - damping_ratio**2)**(1/2))
    proportional_gain = 5 * (natural_frequency**2 - 0.6)
    derivative_gain = 5 * (2 * damping_ratio * natural_frequency - 0.1)
    msd.add_controller(PD.MassSpringDamper, proportional_gain, derivative_gain)
    request = sg.generator(sg.constant, t_step=msd.seconds_per_sim_step, t_final=5)
    handle = msd.view_animation(request)
    Sim.Animations.plt.waitforbuttonpress()
    Sim.Animations.plt.close()


def d_8_b():
    msd = Sim.MassSpringDamper()
    max_force = 2
    damping_ratio = 1 / 2**(1/2)
    # Tuned rise time
    rise_time = 2.2
    natural_frequency = np.pi / (2 * rise_time * (1 - damping_ratio**2)**(1/2))
    proportional_gain = 5 * (natural_frequency**2 - 0.6)
    derivative_gain = 5 * (2 * damping_ratio * natural_frequency - 0.1)
    msd.add_controller(PD.MassSpringDamper, proportional_gain, derivative_gain, max_force)
    request = sg.generator(sg.constant, t_step=msd.seconds_per_sim_step, t_final=5)
    handle = msd.view_animation(request)
    Sim.Animations.plt.waitforbuttonpress()
    Sim.Animations.plt.close()


def e_8_f():
    bnb


if __name__ == '__main__':
    # d_7_d()
    # d_8_a()
    # d_8_b()
    e_8_f()


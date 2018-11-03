import numpy as np

import PIDController as PID
import Simulation as Sim
import Simulation.signal_generator as sg
import Simulation.parameters as params


def d_10():
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


def e_10():
    bnb = Sim.BallAndBeam()
    max_force = 15.0
    damping_ratio = 1 / (2 ** (1 / 2))

    length = bnb.dynamics.beam_length
    ball_mass = bnb.dynamics.ball_mass
    beam_mass = bnb.dynamics.beam_mass
    z_equilibrium = length / 2
    gravity = bnb.dynamics.gravity
    b_theta = length / ((beam_mass * length ** 2) / 3 + ball_mass * z_equilibrium ** 2)

    #  Tuning variables
    rise_time_theta = 1
    bandwidth_separation = 10
    integration_gain_theta = 0
    integration_gain_z = 0
    threshold_theta = np.inf
    threshold_z = np.inf

    natural_frequency_theta = np.pi / (2 * rise_time_theta * (1 - damping_ratio ** 2) ** (1 / 2))
    proportional_gain_theta = natural_frequency_theta ** 2 / b_theta
    derivative_gain_theta = 2 * damping_ratio * natural_frequency_theta / b_theta

    rise_time_z = rise_time_theta * bandwidth_separation
    natural_frequency_z = np.pi / (2 * rise_time_z * (1 - damping_ratio ** 2) ** (1 / 2))
    proportional_gain_z = - natural_frequency_z ** 2 / gravity
    derivative_gain_z = - 2 * damping_ratio * natural_frequency_z / gravity

    bnb.add_controller(PID.BallAndBeam,
                       proportional_gain_z, derivative_gain_z, integration_gain_z,
                       proportional_gain_theta, derivative_gain_theta, integration_gain_theta,
                       threshold_z, threshold_theta, max_force)

    # a square wave with magnitude 0.25±0.15 meters and frequency 0.01 Hz
    requests = sg.generator(sg.square, amplitude=0.15, y_offset=0.25, frequency=0.1,
                            t_step=bnb.seconds_per_sim_step, t_final=30)
    handle = bnb.view_animation(requests)
    Sim.Animations.plt.waitforbuttonpress()
    Sim.Animations.plt.close()
    return handle


def f_10():
    pass


if __name__ == '__main__':
    # d_10()
    e_10()

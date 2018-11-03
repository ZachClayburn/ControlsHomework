import numpy as np

import PIDController as PID
import Simulation as Sim
import Simulation.signal_generator as sg


def d_10():
    msd = Sim.MassSpringDamper()
    max_force = 2.0
    damping_ratio = 1 / (2 ** (1 / 2))

    proportional_gain = max_force
    natural_frequency = ((msd.dynamics.spring_const + proportional_gain) / msd.dynamics.mass) ** (1 / 2)
    derivative_gain = msd.dynamics.mass * 2 * damping_ratio * natural_frequency - msd.dynamics.damping

    integrator_gain = 1

    msd.add_controller(PID.MassSpringDamper, proportional_gain, derivative_gain, integrator_gain, max_force)

    request = sg.generator(sg.constant, amplitude=2/3, t_step=msd.seconds_per_sim_step, t_final=20)

    handle = msd.save_movie(request, 'out/HW7D.mp4')
    # Sim.Animations.plt.waitforbuttonpress()
    # Sim.Animations.plt.close()
    return handle


def e_10():
    bnb = Sim.BallAndBeam()
    max_force = 15.0
    damping_ratio = 1 / (2 ** (1 / 2)) + 0.2

    length = bnb.dynamics.beam_length
    ball_mass = bnb.dynamics.ball_mass
    beam_mass = bnb.dynamics.beam_mass
    z_equilibrium = length / 2
    gravity = bnb.dynamics.gravity
    b_theta = length / ((beam_mass * length ** 2) / 3 + ball_mass * z_equilibrium ** 2)

    #  Tuning variables
    rise_time_theta = .5
    bandwidth_separation = 10
    integration_gain_theta = .01
    integration_gain_z = 0.001
    threshold_theta = 0.0005
    threshold_z = 0.02

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

    # a square wave with magnitude 0.25±0.15 meters and frequency 0.05 Hz
    requests = sg.generator(sg.square, amplitude=0.15, y_offset=0.25, frequency=0.05,
                            t_step=bnb.seconds_per_sim_step, t_final=20)
    handle = bnb.save_movie(requests, 'out/HW7E.mp4')
    # Sim.Animations.plt.waitforbuttonpress()
    # Sim.Animations.plt.close()
    return handle


def f_10():
    pvt = Sim.PlanarVTOL()
    max_force = 10
    damping_ratio = 1 / (2 ** (1 / 2)) + 0.2

    # Tuning variable
    rise_time_h = 2.5
    rise_time_theta = 0.1
    bandwidth_separation = 7
    integration_gain_h = 0.0001
    integration_gain_z = 0.01
    integration_gain_theta = 0.001
    threshold_h = 0.01
    threshold_z = 0.01
    threshold_theta = 0.01

    natural_frequency_h = np.pi / (2 * rise_time_h * (1 - damping_ratio**2)**(1/2))
    c_h = 1 / (pvt.dynamics.center_mass + 2 * pvt.dynamics.wing_mass)
    proportional_gain_h = natural_frequency_h**2 / c_h
    derivative_gain_h = 2 * damping_ratio * natural_frequency_h / c_h

    natural_frequency_theta = np.pi / (2 * rise_time_theta * (1 - damping_ratio**2)**(1/2))
    j_center = pvt.dynamics.center_moi
    wing_mass = pvt.dynamics.wing_mass
    wing_spacing = pvt.dynamics.wing_spacing
    c_theta = 1 / (j_center + 2 * wing_mass * wing_spacing**2)

    proportional_gain_theta = natural_frequency_theta**2 / c_theta
    derivative_gain_theta = 2 * damping_ratio * natural_frequency_theta / c_theta

    rise_time_z = bandwidth_separation * rise_time_theta
    natural_frequency_z = np.pi / (2 * rise_time_z * (1 - damping_ratio**2)**(1/2))
    mu = pvt.dynamics.drag
    gravity = pvt.dynamics.gravity

    proportional_gain_z = natural_frequency_z**2 / -gravity
    derivative_gain_z = (2 * damping_ratio * natural_frequency_z - mu * c_h) / -gravity

    pvt.add_controller(PID.PlanarVTOL,
                       proportional_gain_h, derivative_gain_h, integration_gain_h,
                       proportional_gain_z, derivative_gain_z, integration_gain_z,
                       proportional_gain_theta, derivative_gain_theta, integration_gain_theta,
                       threshold_h, threshold_z, threshold_theta, max_force)
    request = zip(
        sg.generator(sg.constant, y_offset=1, t_step=pvt.seconds_per_sim_step, t_final=20),
        sg.generator(sg.sin, frequency=0.08, y_offset=0, amplitude=2.5, t_step=pvt.seconds_per_sim_step, t_final=20),
    )
    handle = pvt.save_movie(request, 'out/HW7F.mp4')
    # Sim.Animations.plt.waitforbuttonpress()
    # Sim.Animations.plt.close()
    return handle


if __name__ == '__main__':
    d_10()
    e_10()
    f_10()

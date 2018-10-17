import numpy as np

import PDController as PD
import Simulation as Sim
import Simulation.signal_generator as sg

damping_ratio = 1 / 2 ** (1 / 2)


def d_7_d():
    msd = Sim.MassSpringDamper()
    proportional_gain = 4.5
    derivative_gain = 12
    msd.add_controller(PD.MassSpringDamper, proportional_gain, derivative_gain)
    request = sg.generator(sg.constant, t_step=msd.seconds_per_sim_step, t_final=5)
    handle = msd.view_animation(request)
    Sim.Animations.plt.waitforbuttonpress()
    Sim.Animations.plt.close()
    return handle


def d_8_a():
    msd = Sim.MassSpringDamper()
    rise_time = 2
    natural_frequency = np.pi / (2 * rise_time * (1 - damping_ratio**2)**(1/2))
    proportional_gain = 5 * (natural_frequency**2 - 0.6)
    derivative_gain = 5 * (2 * damping_ratio * natural_frequency - 0.1)
    msd.add_controller(PD.MassSpringDamper, proportional_gain, derivative_gain)
    request = sg.generator(sg.constant, t_step=msd.seconds_per_sim_step, t_final=5)
    handle = msd.view_animation(request)
    Sim.Animations.plt.waitforbuttonpress()
    Sim.Animations.plt.close()
    return handle


def d_8_b():
    msd = Sim.MassSpringDamper()
    max_force = 2
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
    return handle


def e_8_e():
    bnb = Sim.BallAndBeam()
    rise_time_theta = 1

    beam_mass = bnb.dynamics.beam_mass
    ball_mass = bnb.dynamics.ball_mass
    beam_length = bnb.dynamics.beam_length
    gravity = bnb.dynamics.gravity

    c_theta = 1 / (beam_length * (ball_mass / 4 + beam_mass / 3))
    natural_frequency_theta = np.pi / (2 * rise_time_theta * (1 - damping_ratio**2)**(1/2))
    proportional_gain_theta = natural_frequency_theta**2 / c_theta
    derivative_gain_theta = 2 * damping_ratio * natural_frequency_theta / c_theta

    rise_time_z = 10 * rise_time_theta
    natural_frequency_z = np.pi / (2 * rise_time_z * (1 - damping_ratio**2)**(1/2))
    proportional_gain_z = natural_frequency_z**2 / -gravity
    derivative_gain_z = 2 * damping_ratio * natural_frequency_z / -gravity

    bnb.add_controller(PD.BallAndBeam,
                       proportional_gain_z, derivative_gain_z,
                       proportional_gain_theta, derivative_gain_theta)
    # a square wave with magnitude 0.25±0.15 meters and frequency 0.01 Hz
    requests = sg.generator(sg.square, amplitude=0.15, y_offset=0.25, frequency=0.01,
                            t_step=bnb.seconds_per_sim_step, t_final=5)
    handle = bnb.view_animation(requests)
    Sim.Animations.plt.waitforbuttonpress()
    Sim.Animations.plt.close()
    return handle


def e_8_f():
    bnb = Sim.BallAndBeam()
    max_force = 15
    rise_time_theta = 1

    beam_mass = bnb.dynamics.beam_mass
    ball_mass = bnb.dynamics.ball_mass
    beam_length = bnb.dynamics.beam_length
    gravity = bnb.dynamics.gravity

    c_theta = 1 / (beam_length * (ball_mass / 4 + beam_mass / 3))
    natural_frequency_theta = np.pi / (2 * rise_time_theta * (1 - damping_ratio**2)**(1/2))
    proportional_gain_theta = natural_frequency_theta**2 / c_theta
    derivative_gain_theta = 2 * damping_ratio * natural_frequency_theta / c_theta

    rise_time_z = 10 * rise_time_theta
    natural_frequency_z = np.pi / (2 * rise_time_z * (1 - damping_ratio**2)**(1/2))
    proportional_gain_z = natural_frequency_z**2 / -gravity
    derivative_gain_z = 2 * damping_ratio * natural_frequency_z / -gravity

    bnb.add_controller(PD.BallAndBeam,
                       proportional_gain_z, derivative_gain_z,
                       proportional_gain_theta, derivative_gain_theta, max_force)
    # a square wave with magnitude 0.25±0.15 meters and frequency 0.01 Hz
    requests = sg.generator(sg.constant, amplitude=0.25, t_step=bnb.seconds_per_sim_step, t_final=5)
    handle = bnb.view_animation(requests)
    Sim.Animations.plt.waitforbuttonpress()
    Sim.Animations.plt.close()
    return handle


def f_7_d():
    pvt = Sim.PlanarVTOL()
    proportional_gain = 0.75
    derivative_gain = 0.09
    pvt.add_controller(PD.PlanarVTOL, proportional_gain, derivative_gain, 0, 0, 0, 0)
    request = zip(
        sg.generator(sg.constant, y_offset=1, t_step=pvt.seconds_per_sim_step, t_final=10),
        sg.generator(sg.constant, y_offset=1, t_step=pvt.seconds_per_sim_step, t_final=10)
    )
    handle = pvt.view_animation(request)
    Sim.Animations.plt.waitforbuttonpress()
    Sim.Animations.plt.close()
    return handle


def f_8_a():
    pvt = Sim.PlanarVTOL()
    rise_time = 8
    natural_frequency = np.pi / (2 * rise_time * (1 - damping_ratio**2)**(1/2))
    c_h = 1 / (pvt.dynamics.center_mass + 2 * pvt.dynamics.wing_mass)
    proportional_gain_h = natural_frequency**2 / c_h
    derivative_gain_h = 2 * damping_ratio * natural_frequency / c_h
    pvt.add_controller(PD.PlanarVTOL, proportional_gain_h, derivative_gain_h, 0, 0, 0, 0)
    request = zip(
        sg.generator(sg.constant, y_offset=1, t_step=pvt.seconds_per_sim_step, t_final=10),
        sg.generator(sg.sin, frequency=0.01, t_step=pvt.seconds_per_sim_step, t_final=10),
    )
    handle = pvt.view_animation(request)
    Sim.Animations.plt.waitforbuttonpress()
    Sim.Animations.plt.close()
    return handle


def f_8_f():
    pvt = Sim.PlanarVTOL()
    rise_time_h = 8
    natural_frequency_h = np.pi / (2 * rise_time_h * (1 - damping_ratio**2)**(1/2))
    c_h = 1 / (pvt.dynamics.center_mass + 2 * pvt.dynamics.wing_mass)
    proportional_gain_h = natural_frequency_h**2 / c_h
    derivative_gain_h = 2 * damping_ratio * natural_frequency_h / c_h

    rise_time_theta = 0.8
    natural_frequency_theta = np.pi / (2 * rise_time_theta * (1 - damping_ratio**2)**(1/2))
    j_center = pvt.dynamics.center_moi
    wing_mass = pvt.dynamics.wing_mass
    wing_spacing = pvt.dynamics.wing_spacing
    c_theta = 1 / (j_center + 2 * wing_mass * wing_spacing**2)

    proportional_gain_theta = natural_frequency_theta**2 / c_theta
    derivative_gain_theta = 2 * damping_ratio * natural_frequency_theta / c_theta

    rise_time_z = 10 * rise_time_theta
    natural_frequency_z = np.pi / (2 * rise_time_z * (1 - damping_ratio**2)**(1/2))
    mu = pvt.dynamics.drag
    gravity = pvt.dynamics.gravity

    proportional_gain_z = natural_frequency_z**2 / -gravity
    derivative_gain_z = (2 * damping_ratio * natural_frequency_z - mu * c_h) / -gravity

    pvt.add_controller(PD.PlanarVTOL,
                       proportional_gain_h, derivative_gain_h,
                       proportional_gain_z, derivative_gain_z,
                       proportional_gain_theta, derivative_gain_theta)
    request = zip(
        sg.generator(sg.constant, y_offset=0, t_step=pvt.seconds_per_sim_step, t_final=10),
        # a square wave with magnitude 3 ± 2.5 meters and frequency 0.08 Hz
        sg.generator(sg.sin, frequency=0.08, y_offset=3, amplitude=2.5, t_step=pvt.seconds_per_sim_step, t_final=10),
    )
    handle = pvt.view_animation(request)
    Sim.Animations.plt.waitforbuttonpress()
    Sim.Animations.plt.close()
    return handle


def f_8_g():
    pvt = Sim.PlanarVTOL()
    rise_time_h = 8
    natural_frequency_h = np.pi / (2 * rise_time_h * (1 - damping_ratio**2)**(1/2))
    c_h = 1 / (pvt.dynamics.center_mass + 2 * pvt.dynamics.wing_mass)
    proportional_gain_h = natural_frequency_h**2 / c_h
    derivative_gain_h = 2 * damping_ratio * natural_frequency_h / c_h

    rise_time_theta = 0.8
    natural_frequency_theta = np.pi / (2 * rise_time_theta * (1 - damping_ratio**2)**(1/2))
    j_center = pvt.dynamics.center_moi
    wing_mass = pvt.dynamics.wing_mass
    wing_spacing = pvt.dynamics.wing_spacing
    c_theta = 1 / (j_center + 2 * wing_mass * wing_spacing**2)

    proportional_gain_theta = natural_frequency_theta**2 / c_theta
    derivative_gain_theta = 2 * damping_ratio * natural_frequency_theta / c_theta

    rise_time_z = 10 * rise_time_theta
    natural_frequency_z = np.pi / (2 * rise_time_z * (1 - damping_ratio**2)**(1/2))
    mu = pvt.dynamics.drag
    gravity = pvt.dynamics.gravity

    proportional_gain_z = natural_frequency_z**2 / -gravity
    derivative_gain_z = (2 * damping_ratio * natural_frequency_z - mu * c_h) / -gravity

    pvt.add_controller(PD.PlanarVTOL,
                       proportional_gain_h, derivative_gain_h,
                       proportional_gain_z, derivative_gain_z,
                       proportional_gain_theta, derivative_gain_theta)
    request = zip(
        sg.generator(sg.constant, y_offset=0, t_step=pvt.seconds_per_sim_step, t_final=10),
        # a square wave with magnitude 3 ± 2.5 meters and frequency 0.08 Hz
        sg.generator(sg.sin, frequency=0.08, y_offset=3, amplitude=2.5, t_step=pvt.seconds_per_sim_step, t_final=10),
    )
    handle = pvt.view_animation(request)
    Sim.Animations.plt.waitforbuttonpress()
    Sim.Animations.plt.close()
    return handle


if __name__ == '__main__':
    # d_7_d()
    # d_8_a()
    # d_8_b()
    e_8_e()
    e_8_f()
    # f_7_d()
    # f_8_a()
    # f_8_f()
    # f_8_g()

from typing import Tuple

import Simulation as Sim
import control.matlab as ctrl
import numpy as np
import Simulation.signal_generator as sg
import SSIController as Control


def augment_matrices(A: np.ndarray, B: np.ndarray, C: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    temp = np.concatenate((A, C))
    A_I = np.concatenate((temp, np.zeros((temp.shape[0], 1))), 1)

    B_I = np.concatenate((B, np.zeros((1, 1))))

    return A_I, B_I


def d_12():
    print('\n--D--\n')
    msd = Sim.MassSpringDamper()
    max_force = 2.0
    damping_ratio = 1 / (2 ** (1 / 2))

    rise_time = 2.0

    # mass = msd.dynamics.mass
    # spring_const = msd.dynamics.spring_const
    # natural_frequency = ((spring_const + max_force) / mass) ** (1 / 2)
    natural_frequency = np.pi / (2 * rise_time * (1 - damping_ratio ** 2) ** (1 / 2))

    coefs = [1, 2 * damping_ratio * natural_frequency, natural_frequency ** 2]
    integrator_root = 1.0
    roots = np.roots(np.convolve(coefs, [1, integrator_root]))
    print(f"Roots: {roots}")

    # Part B
    A = msd.dynamics.A
    B = msd.dynamics.B
    C = msd.dynamics.C

    A_I, B_I = augment_matrices(A, B, C)

    gains = ctrl.place(A_I, B_I, roots)
    feedback_gain = gains[:, 0:2]
    integrator_gain = gains[0, 2]
    print(f'Feedback Gain: {feedback_gain}\nIntegrator Gain: {integrator_gain}')

    msd.add_controller(Control.MassSpringDamper,
                       feedback_gain, integrator_gain, max_force)
    request = sg.generator(sg.constant, amplitude=2/3, t_step=msd.seconds_per_sim_step, t_final=20)
    handle = msd.view_animation(request)
    Sim.Animations.plt.waitforbuttonpress()
    Sim.Animations.plt.close()

    return handle


def e_12():
    print('\n--E--\n')
    bnb = Sim.BallAndBeam()
    max_force = 15
    damping_ratio = 1 / (2 ** (1 / 2))

    #  Tuning variables
    rise_time_theta = .5
    bandwidth_separation = 2.4
    integrator_pole = 2.

    natural_frequency_theta = np.pi / (2 * rise_time_theta * (1 - damping_ratio ** 2) ** (1 / 2))
    rise_time_z = rise_time_theta * bandwidth_separation
    natural_frequency_z = np.pi / (2 * rise_time_z * (1 - damping_ratio ** 2) ** (1 / 2))

    coefs_theta = [1, 2 * damping_ratio * natural_frequency_theta, natural_frequency_theta ** 2]
    coefs_z = [1, 2 * damping_ratio * natural_frequency_z, natural_frequency_z ** 2]
    coefs_I = [1, integrator_pole]
    roots = np.roots(np.convolve(coefs_I, np.convolve(coefs_theta, coefs_z)))

    A = bnb.dynamics.A
    B = bnb.dynamics.B
    C = bnb.dynamics.C

    A_I, B_I = augment_matrices(A, B, C[0:1, :])

    gains = ctrl.place(A_I, B_I, roots)
    dim = A.shape[1]
    feedback_gain = gains[0:1, 0:dim]
    integrator_gain = gains[0, dim]
    print(f'Feedback Gain: {feedback_gain}\nIntegrator Gain: {integrator_gain}')

    bnb.add_controller(Control.BallAndBeam, feedback_gain, integrator_gain, max_force=max_force)
    requests = sg.generator(sg.square, amplitude=0.15, y_offset=0.25, frequency=0.1,
                            t_step=bnb.seconds_per_sim_step, t_final=90)

    handle = bnb.view_animation(requests)
    Sim.Animations.plt.waitforbuttonpress()
    Sim.Animations.plt.close()
    return handle


def f_12():
    print('\n--F--\n')
    pvt = Sim.PlanarVTOL()
    max_force = 10
    damping_ratio = 1 / (2 ** (1 / 2)) + 0.2

    # Tuning variable
    rise_time_h = 2.5
    rise_time_theta = 0.1
    bandwidth_separation = 7
    integrator_pole_lat = 1.5
    integrator_pole_lon = 1

    natural_frequency_h = np.pi / (2 * rise_time_h * (1 - damping_ratio**2)**(1/2))
    natural_frequency_theta = np.pi / (2 * rise_time_theta * (1 - damping_ratio**2)**(1/2))
    rise_time_z = bandwidth_separation * rise_time_theta
    natural_frequency_z = np.pi / (2 * rise_time_z * (1 - damping_ratio**2)**(1/2))

    coefs_theta = [1, 2 * damping_ratio * natural_frequency_theta, natural_frequency_theta ** 2]
    coefs_z = [1, 2 * damping_ratio * natural_frequency_z, natural_frequency_z ** 2]
    coefs_I_lat = [1, integrator_pole_lat]
    roots_lat = np.roots(np.convolve(coefs_I_lat, np.convolve(coefs_theta, coefs_z)))
    print(f"Latitudinal roots: {roots_lat}")

    coefs_h = [1, 2 * damping_ratio * natural_frequency_h, natural_frequency_h * 2]
    coefs_I_lon = [1, integrator_pole_lon]
    roots_lon = np.roots(np.convolve(coefs_h, coefs_I_lon))
    print(f"Longitudinal roots: {roots_lon}")

    # Part B
    A_lat = pvt.dynamics.A_lat
    B_lat = pvt.dynamics.B_lat
    C_lat = pvt.dynamics.C_lat

    A_I_lat, B_I_lat = augment_matrices(A_lat, B_lat, C_lat[0:1, :])

    A_lon = pvt.dynamics.A_lon
    B_lon = pvt.dynamics.B_lon
    C_lon = pvt.dynamics.C_lon

    A_I_lon, B_I_lon = augment_matrices(A_lon, B_lon, C_lon)

    gains_lat = ctrl.place(A_I_lat, B_I_lat, roots_lat)
    dim_lat = A_lat.shape[1]
    feedback_gain_lat = gains_lat[0:1, 0:dim_lat]
    int_gain_lat = gains_lat[0, dim_lat]
    print(f'Latitudinal feedback Gain: {feedback_gain_lat}\nLatitudinal integrator gain: {int_gain_lat}')

    gains_lon = ctrl.place(A_I_lon, B_I_lon, roots_lon)
    dim_lon = A_lon.shape[1]
    feedback_gain_lon = gains_lon[0:1, 0:dim_lon]
    int_gain_lon = gains_lon[0, dim_lon]
    print(f'Longitudinal feedback Gain: {feedback_gain_lon}\nLongitudinal integrator gain: {int_gain_lon}')

    request = zip(
        sg.generator(sg.constant, y_offset=1, t_step=pvt.seconds_per_sim_step, t_final=20),
        sg.generator(sg.sin, frequency=0.08, y_offset=0, amplitude=2.5, t_step=pvt.seconds_per_sim_step, t_final=20),
    )

    pvt.add_controller(Control.PlanarVTOL,
                       feedback_gain_lat, int_gain_lat,
                       feedback_gain_lon, int_gain_lon,
                       max_force)

    handel = pvt.view_animation(request)
    Sim.Animations.plt.waitforbuttonpress()
    Sim.Animations.plt.close()
    return handel


if __name__ == '__main__':
    # d_12()
    # e_12()
    f_12()

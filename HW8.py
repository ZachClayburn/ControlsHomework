import Simulation as Sim
import control.matlab as ctrl
import numpy as np
import Simulation.signal_generator as sg
import SSController


def d_11():
    print('\n--D--\n')
    msd = Sim.MassSpringDamper()
    max_force = 2.0
    damping_ratio = 1 / (2 ** (1 / 2))

    mass = msd.dynamics.mass
    spring_const = msd.dynamics.spring_const
    natural_frequency = ((spring_const + max_force) / mass) ** (1 / 2)

    # Part A
    coefs = [1, 2 * damping_ratio * natural_frequency, natural_frequency * 2]
    roots = np.roots(coefs)
    print(f"Roots: {roots}")

    # Part B
    A = msd.dynamics.A
    B = msd.dynamics.B
    C = msd.dynamics.C

    # Part C
    control_mat = ctrl.ctrb(A, B)
    rank = np.linalg.matrix_rank(control_mat)
    order = A.shape[0]
    print(f"controllability matrix rank: {rank}")
    print(f"System order: {order}")
    print(f"{'Controllable' if rank == order else 'Not Controllable'}")

    # Part D
    feedback_gain = ctrl.place(A, B, roots)
    print(f'Feedback Gain: {feedback_gain}')

    reference_gain = -1 / (C @ np.linalg.inv(A - B @ feedback_gain) @ B)
    print(f"Reference gain: {reference_gain}")

    # Part E
    msd.add_controller(SSController.MassSpringDamper,
                       feedback_gain, reference_gain, max_force)
    request = sg.generator(sg.constant, amplitude=2/3, t_step=msd.seconds_per_sim_step, t_final=20)
    handle = msd.view_animation(request)
    Sim.Animations.plt.waitforbuttonpress()
    Sim.Animations.plt.close()


def e_11():
    print('\n--E--\n')
    bnb = Sim.BallAndBeam()
    max_force = 15
    damping_ratio = 1 / (2 ** (1 / 2))

    #  Tuning variables
    rise_time_theta = .5
    bandwidth_separation = 10

    natural_frequency_theta = np.pi / (2 * rise_time_theta * (1 - damping_ratio ** 2) ** (1 / 2))
    rise_time_z = rise_time_theta * bandwidth_separation
    natural_frequency_z = np.pi / (2 * rise_time_z * (1 - damping_ratio ** 2) ** (1 / 2))

    # Part A
    coefs_theta = [1, 2 * damping_ratio * natural_frequency_theta, natural_frequency_theta ** 2]
    coefs_z = [1, 2 * damping_ratio * natural_frequency_z, natural_frequency_z ** 2]
    roots = np.concatenate((
        np.roots(coefs_theta),
        np.roots(coefs_z),
    ))

    # Part B
    A = bnb.dynamics.A
    B = bnb.dynamics.B
    C = bnb.dynamics.C

    # Part C
    control_mat = ctrl.ctrb(A, B)
    rank = np.linalg.matrix_rank(control_mat)
    order = A.shape[0]
    print(f"controllability matrix rank: {rank}")
    print(f"System order: {order}")
    print(f"{'Controllable' if rank == order else 'Not Controllable'}")

    # Part D
    feedback_gain = ctrl.place(A, B, roots)
    print(f'Feedback Gain: {feedback_gain}')

    reference_gain = -1 / (C[0] @ np.linalg.inv(A - B @ feedback_gain) @ B)
    print(f"Reference gain: {reference_gain}")

    # Part E
    bnb.add_controller(SSController.BallAndBeam, feedback_gain, reference_gain, max_force)
    requests = sg.generator(sg.square, amplitude=0.15, y_offset=0.25, frequency=0.05,
                            t_step=bnb.seconds_per_sim_step, t_final=20)

    handle = bnb.view_animation(requests)
    Sim.Animations.plt.waitforbuttonpress()
    Sim.Animations.plt.close()
    return handle


def f_11():
    print('\n--F--\n')
    pvt = Sim.PlanarVTOL()
    max_force = 10
    damping_ratio = 1 / (2 ** (1 / 2)) + 0.2

    # Tuning variable
    rise_time_h = 2.5
    rise_time_theta = 0.1
    bandwidth_separation = 7

    natural_frequency_h = np.pi / (2 * rise_time_h * (1 - damping_ratio**2)**(1/2))
    natural_frequency_theta = np.pi / (2 * rise_time_theta * (1 - damping_ratio**2)**(1/2))
    rise_time_z = bandwidth_separation * rise_time_theta
    natural_frequency_z = np.pi / (2 * rise_time_z * (1 - damping_ratio**2)**(1/2))

    # Part A
    coefs_theta = [1, 2 * damping_ratio * natural_frequency_theta, natural_frequency_theta ** 2]
    coefs_z = [1, 2 * damping_ratio * natural_frequency_z, natural_frequency_z ** 2]
    roots_lat = np.concatenate((
        np.roots(coefs_theta),
        np.roots(coefs_z),
    ))
    print(f"Latitudinal roots: {roots_lat}")

    coefs_h = [1, 2 * damping_ratio * natural_frequency_h, natural_frequency_h * 2]
    roots_lon = np.roots(coefs_h)
    print(f"Longitudinal roots: {roots_lon}")

    # Part B
    A_lat = pvt.dynamics.A_lat
    B_lat = pvt.dynamics.B_lat
    C_lat = pvt.dynamics.C_lat

    A_lon = pvt.dynamics.A_lon
    B_lon = pvt.dynamics.B_lon
    C_lon = pvt.dynamics.C_lon

    # Part C
    control_mat_lat = ctrl.ctrb(A_lat, B_lat)
    rank = np.linalg.matrix_rank(control_mat_lat)
    order = A_lat.shape[0]
    print(f"Latitudinal controllability matrix rank: {rank}")
    print(f"Latitudinal system order: {order}")
    print(f"{'Controllable' if rank == order else 'Not Controllable'}")

    control_mat_lon = ctrl.ctrb(A_lon, B_lon)
    rank = np.linalg.matrix_rank(control_mat_lon)
    order = A_lon.shape[0]
    print(f"Longitudinal Controllability matrix rank: {rank}")
    print(f"Longitudinal system order: {order}")
    print(f"{'Controllable' if rank == order else 'Not Controllable'}")


    # Part D
    feedback_gain_lat = ctrl.place(A_lat, B_lat, roots_lat)
    print(f'Latitudinal feedback Gain: {feedback_gain_lat}')

    reference_gain_lat = -1 / (C_lat[0] @ np.linalg.inv(A_lat - B_lat @ feedback_gain_lat) @ B_lat)
    print(f"Latitudinal reference gain: {reference_gain_lat}")

    feedback_gain_lon = ctrl.place(A_lon, B_lon, roots_lon)
    print(f'Longitudinal feedback Gain: {feedback_gain_lon}')

    reference_gain_lon = -1 / (C_lon @ np.linalg.inv(A_lon - B_lon @ feedback_gain_lon) @ B_lon)
    print(f"Longitudinal reference gain: {reference_gain_lon}")

    # Part E
    request = zip(
        sg.generator(sg.constant, y_offset=1, t_step=pvt.seconds_per_sim_step, t_final=20),
        sg.generator(sg.sin, frequency=0.08, y_offset=0, amplitude=2.5, t_step=pvt.seconds_per_sim_step, t_final=20),
    )

    pvt.add_controller(SSController.PlanarVTOL,
                       feedback_gain_lat, reference_gain_lat,
                       feedback_gain_lon, reference_gain_lon,
                       max_force)

    handel = pvt.view_animation(request)
    Sim.Animations.plt.waitforbuttonpress()
    Sim.Animations.plt.close()
    return handel


if __name__ == '__main__':
    # d_11()
    # e_11()
    f_11()

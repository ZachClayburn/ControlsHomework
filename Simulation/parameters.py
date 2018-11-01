import random


randomness_parameter = 0.0
upper = 1 + randomness_parameter
lower = 1 - randomness_parameter


def _perturbation(value: float):
    return value * random.uniform(lower, upper)


class GlobalParameters:
    sample_rate = 0.04
    sample_rate_real = _perturbation(sample_rate)
    gravity = 9.8


class MassSpringDamper:
    width = 1.0
    height = 1.0
    mass = 5.0
    mass_real = _perturbation(mass)
    spring_const = 3.0
    spring_const_real = _perturbation(spring_const)
    damping = 0.5
    damping_real = _perturbation(damping)
    state_0 = [
        [0.0],
        [0.0]
    ]
    animation_0 = state_0[0]


class BallAndBeam:
    beam_length = 0.5
    beam_length_real = _perturbation(beam_length)
    beam_width = 0.01
    beam_mass = 2.0
    beam_mass_real = _perturbation(beam_mass)
    ball_radius = 0.06
    ball_mass = 0.35
    ball_mass_real = _perturbation(ball_mass)
    state_0 = [
        [beam_length / 2],
        [0.0],
        [0.0],
        [0.0]
    ]
    animation_0 = [state_0[0][0], state_0[1][0]]


class PlanarVTOL:
    body_width = 0.2
    body_height = 0.125
    wing_major_axis = 0.25
    wing_minor_axis = 0.1
    wing_spacing = 0.3
    wing_spacing_real = _perturbation(wing_spacing)
    center_mass = 1.0
    center_mass_real = _perturbation(center_mass)
    center_moi = 0.0042
    center_moi_real = _perturbation(center_moi)
    wing_mass = 0.25
    drag = 0.1
    drag_real = _perturbation(drag)
    state_0 = [
        [0.0],
        [1.0],
        [0.0],
        [0.0],
        [0.0],
        [0.0],
    ]
    z_target_0 = 0
    animation_0 = [
        state_0[0][0],
        z_target_0,
        state_0[1][0],
        state_0[2][0],
    ]

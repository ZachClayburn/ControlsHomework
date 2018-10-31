import random


randomness_parameter = 0.0
upper = 1 + randomness_parameter
lower = 1 - randomness_parameter


class GlobalParameters:
    sample_rate = 0.01
    gravity = 9.8


class MassSpringDamper:
    width = 1.0
    height = 1.0
    mass = 5.0 * random.uniform(lower, upper)
    spring_const = 3.0 * random.uniform(lower, upper)
    damping = 0.5 * random.uniform(lower, upper)
    state_0 = [
        [0.0],
        [0.0]
    ]
    animation_0 = state_0[0]


class BallAndBeam:
    beam_length = 0.5 * random.uniform(lower, upper)
    beam_width = 0.01
    beam_mass = 2.0 * random.uniform(lower, upper)
    ball_radius = 0.06
    ball_mass = 0.35 * random.uniform(lower, upper)
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
    wing_spacing = 0.3 * random.uniform(lower, upper)
    center_mass = 1.0 * random.uniform(lower, upper)
    center_moi = 0.0042 * random.uniform(lower, upper)
    wing_mass = 0.25
    drag = 0.1 * random.uniform(lower, upper)
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

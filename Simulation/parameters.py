class GlobalParameters:
    sample_rate = 0.04
    gravity = 9.8


class MassSpringDamper:
    width = 1.0
    height = 1.0
    mass = 5.0
    spring_const = 3.0
    damping = 0.5
    state_0 = [
        [0.0],
        [0.0]
    ]
    animation_0 = state_0[0]


class BallAndBeam:
    beam_length = 0.5
    beam_width = 0.01
    beam_mass = 2.0
    ball_radius = 0.06
    ball_mass = 0.35
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
    center_mass = 1.0
    center_moi = 0.0042
    wing_mass = 0.25
    drag = 0.1
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

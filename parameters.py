GlobalParameters = {
    'sample_rate': 0.04
}

MassSpringDamper = {
    'width': 1.0,
    'height': 1.0,
    'mass': 5.0,
    'spring_const': 3.0,
    'damping': 0.5,
    'z_0': 0.0,
    'zdot_0': 0.0,
}


BallAndBeam = {
    'beamLength': 1.0,
    'beamWidth': 0.01,
    'ballRadius': 0.1,
}


PlanarVTOL = {
    'bodyWidth': 0.2,
    'bodyHeight': 0.125,
    'wingSpacing': 0.3,
    'wingMajorAxis': 0.25,
    'wingMinorAxis': 0.1,
}
# TODO Make these each into a namedtuple

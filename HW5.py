import Simulation as Sim
import PDController as PD
import Simulation.signal_generator as sg

if __name__ == '__main__':
    msd = Sim.MassSpringDamper()
    proportional_gain = 1
    derivative_gain = 1
    msd.add_controller(PD.MassSpringDamper, proportional_gain, derivative_gain)
    request = sg.generator(sg.constant, t_step=msd.seconds_per_sim_step, t_final=5)
    handle = msd.view_animation(request)
    Sim.Animations.plt.waitforbuttonpress()


import matplotlib.pyplot as plt
import matplotlib.patches as patches

import parameters as param


class Animation:
    """
    An abstract class for all animation classes
    """

    def __init__(self):
        self._do_init = True
        self.figure, self.axis = plt.subplots()


class MassSpringDamper(Animation):

    def __init__(self):
        Animation.__init__(self)
        self._width = param.MassSpringDamper.WIDTH
        self._height = param.MassSpringDamper.HEIGHT
        self.mass = None

    def draw(self, z_pos):
        xy = (z_pos, 0)
        if self._do_init:
            self.mass = patches.Rectangle(xy, width=param.MassSpringDamper.WIDTH, height=param.MassSpringDamper.HEIGHT)
            self.axis.set_xlim(left=-5, right=5)
            self.axis.set_ylim(bottom=-5, top=5)
            self.axis.add_patch(self.mass)
            self._do_init = False
        else:
            self.mass.xy = xy


if __name__ == '__main__':
    a = MassSpringDamper()
    i = 0
    a.draw(i)
    plt.waitforbuttonpress()
    plt.close()

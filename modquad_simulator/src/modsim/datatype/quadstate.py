
class QuadState(object):
    def __init__(self, pos=[0., 0., 0.], vel=[0., 0., 0.], euler=[0., 0., 0.], omega=[0., 0., 0.]):
        self.pos = pos
        self.vel = vel
        self.euler = euler
        self.omega = omega

        self.pos_des = None
        self.vel_des = None
        self.acc_des = None
        self.yaw_des = None

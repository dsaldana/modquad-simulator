import numpy as np
import itertools

from modsim import params


class Structure:
    max_id = 0

    def __init__(self, ids=['modquad01'], xx=[0], yy=[0], motor_failure=[]):
        """
        :param ids: robot ids
        :param xx: module locations in the structure frame (x-coordinates)
        :param yy: module locations in the structure frame (y-coordinates)
        :param motor_failure: motor failures as a set of tuples, (module from 0 to n-1, rotor number from 0 to 3)
        """
        self.struc_id = Structure.max_id
        Structure.max_id += 1
        self.ids = ids
        self.xx = np.array(xx)
        self.yy = np.array(yy)
        self.motor_failure = motor_failure
        self.motor_roll = [[0, 0, 0, 0], [0, 0, 0, 0]]
        self.motor_pitch = [[0, 0, 0, 0], [0, 0, 0, 0]]

        np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})
        ##
        self.n = len(self.ids)  # Number of modules
        #print(self.xx)
        #print(self.yy)
        #print(np.mean(self.xx))
        #print(np.mean(self.yy))
        self.xx = np.array(self.xx) - np.average(self.xx)# x-coordinates with respect to the center of mass
        self.yy = np.array(self.yy) - np.average(self.yy)# y-coordinates with respect to the center of mass
        #print(self.xx)
        #print(self.yy)

        # Equation (4) of the Modquad paper
        # FIXME inertia with parallel axis theorem is not working. Temporary multiplied by zero
        self.inertia_tensor = 0.5 * self.n * np.array(params.I) + 0.01 * params.mass * np.diag([
            np.sum(self.yy ** 2),
            np.sum(self.xx ** 2),
            np.sum(self.yy ** 2) + np.sum(self.xx ** 2)
        ])

        # self.inertia_tensor = np.array(params.I)
        self.inverse_inertia = np.linalg.inv(self.inertia_tensor)

    def gen_hashstring(self):
        """ 
        This is for reconfig, where we need to determine whether to split structure based on
        the shape of it and the faults it contains
        """
        # Import here in case something else using structure does not need mqscheduler package
        from modquad_sched_interface.interface import convert_struc_to_mat

        pi = convert_struc_to_mat([int(mid[7:]) for mid in self.ids], self.xx, self.yy)
        R = [r for r in range(pi.shape[0])]
        C = [c for c in range(pi.shape[1])]
        RC = [p for p in itertools.product(R,C)]
        #shape = ';'.join(''.join('%d' % int(x>-1) for x in y) for y in pi)
        shape2 = []
        for r in R:
            for c in C:
                if pi[r,c] != -1:
                    shape2.append('1')
                else:
                    shape2.append('0')
            if r < R[-1]:
                shape2.append(';')
        shape = ''.join(shape2)
        # NOTE: range(4) should be range(num_rotor) for however many rotors the system has, we just use quadrotors
        rotorstat = ','.join(
                        ''.join('%d' % int((int(mod[7:]), rot) not in self.motor_failure) 
                        for rot in range(4)) 
                    for mod in sorted(self.ids))
        print(pi)
        #print(self.ids)
        #print(self.xx)
        #print(self.yy)
        #print("rot_fails: {}".format(self.motor_failure))
        #print(shape + '_' + rotorstat)
        return shape + '_' + rotorstat

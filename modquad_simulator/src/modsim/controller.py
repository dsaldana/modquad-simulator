import modsim.params as params
from math import sin, cos
import numpy as np
from math import sqrt

# Global var will not work with multiple structures
#accumulated_error = np.array([0., 0., 0.])

def position_controller(struc_mgr, struc_ind, desired_state):
    # Import here to avoid circ dependency issue
    from modsim.datatype.structure_manager import StructureManager
    """
    PD controller to convert from position to accelerations, and accelerations to attitude.
    Controller: Using these current and desired states, you have to compute the desired controls
    :param struc_mgr: StructureManager object
    :param qd: The object qt contains the current state and the desired state:
                * The current states are: qd.pos, qd.vel, qd.euler = [rollpitchyaw], qd.omega.
                * The desired states are: qd.pos_des, qd.vel_des, qd.acc_des, qd.yaw_des, qd.yawdot_des
    :return: desired thrust and attitude
    """
    #global accumulated_error
    state_vector = struc_mgr.strucs[struc_ind].state_vector

    # Desired state
    [pos_des, vel_des, acc_des, yaw_des, des_yawdot] = desired_state

    # current state
    pos = state_vector[:3]
    vel = state_vector[3:6]

    # constants
    m = params.mass
    g = params.grav

    xyp =   7.0 #355.0
    xyd =  90.0 #255.0
    xyi =  01.0 #145.0
    zp  =  25.0 #1925.0
    zd  =  18.0 #  0.0
    zi  =   2.5 # 45.0
    kp1_u, kd1_u, ki1_u = xyp, xyd, xyi #10., 71., .0
    kp2_u, kd2_u, ki2_u = xyp, xyd, xyi #10., 71., .0
    kp3_u, kd3_u, ki3_u =  zp,  zd,  zi #10., 48., .0

    # Error
    pos_error = pos_des - pos
    vel_error = vel_des - vel
    struc_mgr.strucs[struc_ind].accumulated_error += pos_error
    #print(pos_error)

    # Desired acceleration
    r1_acc = kp1_u * pos_error[0] + kd1_u * vel_error[0] + acc_des[0] + ki1_u * struc_mgr.strucs[struc_ind].accumulated_error[0]
    r2_acc = kp2_u * pos_error[1] + kd2_u * vel_error[1] + acc_des[1] + ki2_u * struc_mgr.strucs[struc_ind].accumulated_error[1]
    r3_acc = kp3_u * pos_error[2] + kd3_u * vel_error[2] + acc_des[2] + ki3_u * struc_mgr.strucs[struc_ind].accumulated_error[2]

    phi_des = (r1_acc * sin(yaw_des) - r2_acc * cos(yaw_des)) / g
    theta_des = (r1_acc * cos(yaw_des) + r2_acc * sin(yaw_des)) / g
    psi_des = yaw_des

    # Thrust
    thrust = m * g + m * r3_acc
    #print(pos_des, pos)
    #print(pos_error, thrust)
    #print('---')

    # desired thrust and attitude
    return [thrust, phi_des, theta_des, psi_des]


def modquad_torque_control(F, M, structure, motor_sat=False):
    """
    This function is similar to crazyflie_motion, but it is made for modular robots. So it specifies the dynamics
    of the modular structure. It receives a desired force and moment of a single robot.
    :param structure:
    :param motor_sat: motor saturation
    :param F: desired total thrust, float
    :param M: desired moments, 3 x 1 float vector
    :return: thrust and moments for the whole structure
    """
    ## From moments to rotor forces (power distribution)
    # positions of the rotors
    #         ^ X
    #    (4)  |      (1) [L, -L]
    #   Y<-----
    #    (3)         (2)
    rx, ry = [], []
    L = params.arm_length * sqrt(2) / 2.

    for x, y in zip(structure.xx, structure.yy):
        rx.append(x + L)
        rx.append(x - L)
        rx.append(x - L)
        rx.append(x + L)
        # y-axis
        ry.append(y - L)
        ry.append(y - L)
        ry.append(y + L)
        ry.append(y + L)

    sign_rx = [1 if rx_i > 0 else -1 for rx_i in rx]
    sign_ry = [1 if ry_i > 0 else -1 for ry_i in ry]
    # m = 4 * n  # Number of rotors
    A = [[0.25, sy * .25 / L, -sx * .25 / L] for sx, sy in zip(sign_rx, sign_ry)]

    rotor_forces = np.dot(A, [F, M[0], M[1]])  # Not using moment about Z-axis for limits

    # Failing motors -- IDs are 1-indexed, but rotor pos are 0-indexed
    for i, mf in enumerate(sorted(structure.motor_failure)):
        try:
            #print("Fail rotor: {}".format(4*i + mf[1]))
            rotor_forces[4 * i + mf[1]] *= 0.0
        except:
            print(structure.ids)
            print(structure.xx)
            print(structure.yy)
            print(structure.motor_failure)
            print(np.array(A))
            print(np.array([F,M[0],M[1]]))
            print(rotor_forces)
            import sys
            sys.exit(-1)


    # Motor saturation
    if motor_sat:
        rotor_forces[rotor_forces > params.maxF / 4] = params.maxF / 4
        rotor_forces[rotor_forces < params.minF / 4] = params.minF / 4

    # From prop forces to total moments. Equation (1) of the modquad paper (ICRA 18)
    F = np.sum(rotor_forces)
    Mx = np.dot(ry, rotor_forces)
    My = -np.dot(rx, rotor_forces)
    # TODO Mz
    Mz = M[2]

    #print(structure.ids)
    #print(F)
    #print(Mx)
    #print(My)
    #print(rotor_forces)
    #print('---')

    return F, [Mx, My, Mz], rotor_forces

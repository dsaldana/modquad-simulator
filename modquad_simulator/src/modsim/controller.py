import params
from math import sin, cos
import numpy as np


def position_controller(state_vector, desired_state):
    """
    PD controller to convert from position to accelerations, and accelerations to attitude.
    Controller: Using these current and desired states, you have to compute the desired controls

    :param qd: The object qt contains the current state and the desired state:
                * The current states are: qd.pos, qd.vel, qd.euler = [rollpitchyaw], qd.omega.
                * The desired states are: qd.pos_des, qd.vel_des, qd.acc_des, qd.yaw_des, qd.yawdot_des
    :return: desired thrust and attitude
    """

    # Desired state
    [pos_des, vel_des, acc_des, yaw_des, des_yawdot] = desired_state

    # current state
    pos = state_vector[:3]
    vel = state_vector[3:6]

    # constants
    m = params.mass
    g = params.grav

    kp1_u, kd1_u = 10, 71
    kp2_u, kd2_u = 10, 71
    kp3_u, kd3_u = 10, 48

    r1_acc = kp1_u * (pos_des[0] - pos[0]) + kd1_u * (vel_des[0] - vel[0]) + acc_des[0]
    r2_acc = kp2_u * (pos_des[1] - pos[1]) + kd2_u * (vel_des[1] - vel[1]) + acc_des[1]
    r3_acc = kp3_u * (pos_des[2] - pos[2]) + kd3_u * (vel_des[2] - vel[2]) + acc_des[2]

    phi_des = (r1_acc * sin(yaw_des) - r2_acc * cos(yaw_des)) / g
    theta_des = (r1_acc * cos(yaw_des) + r2_acc * sin(yaw_des)) / g
    psi_des = yaw_des

    # Thrust
    thrust = m * g + m * r3_acc

    # desired thrust and attitude
    return [thrust, phi_des, theta_des, psi_des]

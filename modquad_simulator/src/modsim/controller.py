import params
from math import sin, cos
import numpy as np


def control_handle(qd):
    """
    Controller: Using these current and desired states, you have to compute the desired controls

    :param qd: The object qt contains the current state and the desired state:
                * The current states are: qd.pos, qd.vel, qd.euler = [rollpitchyaw], qd.omega.
                * The desired states are: qd.pos_des, qd.vel_des, qd.acc_des, qd.yaw_des, qd.yawdot_des
    :return:
    """
    # =================== Your code goes here ================================

    m = params.mass
    g = params.grav

    kp1_u, kd1_u = 10, 71
    kp2_u, kd2_u = 10, 71
    kp3_u, kd3_u = 10, 48

    kp_fi, kd_fi = 2000, 125
    kp_theta, kd_theta = 2000, 125
    kp_yaw, kd_yaw = 1, 0.05

    r1_acc = kp1_u * (qd.pos_des[0] - qd.pos[0]) + kd1_u * (qd.vel_des[0] - qd.vel[0]) + qd.acc_des[0]
    r2_acc = kp2_u * (qd.pos_des[1] - qd.pos[1]) + kd2_u * (qd.vel_des[1] - qd.vel[1]) + qd.acc_des[1]
    r3_acc = kp3_u * (qd.pos_des[2] - qd.pos[2]) + kd3_u * (qd.vel_des[2] - qd.vel[2]) + qd.acc_des[2]

    phi_des = (r1_acc * sin(qd.yaw_des) - r2_acc * cos(qd.yaw_des)) / g
    theta_des = (r1_acc * cos(qd.yaw_des) + r2_acc * sin(qd.yaw_des)) / g
    psi_des = qd.yaw_des

    p_des = 0
    q_des = 0
    r_des = qd.yawdot_des

    # Thrust

    u1 = m * g + m * r3_acc
    F = u1

    # Moment
    u2 = np.dot(params.I, [kp_fi * (phi_des - qd.euler[0]) + kd_fi * (p_des - qd.omega[0]),
                           kp_theta * (theta_des - qd.euler[1]) + kd_theta * (q_des - qd.omega[1]),
                           kp_yaw * (psi_des - qd.euler[2]) + kd_yaw * (r_des - qd.omega[2])])

    M = u2

    # M    = [0 0 0]' # You should fill this in
    # =================== Your code ends here ===================

    # Output trpy and drpy as in hardware
    trpy = [F, phi_des, theta_des, psi_des]
    drpy = [0, 0, 0, 0]

    return [F, M, trpy, drpy]

import params
from math import sin, cos
import numpy as np
from math import sqrt


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


def modquad_torque_control(F, M, structure, motor_sat=False):
    """
    This function is similar to crazyflie_motion, but it is made for modular robots. So it specifies the dynamics
    of the modular structure. It receives a desired force and moment of a single robot.
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
    # Failing motors
    for mf in structure.motor_failure:
        # rotor_forces[4 * mf[0] + mf[1]] = 0.0
        rotor_forces[4 * mf[0] + mf[1]] *= 0.99  # TODO failure is only simulating 90%

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

    return F, [Mx, My, Mz], rotor_forces





# def crazyflie_torquecontrol(F, M):
#     """
#     It receives a desired force and moment. The equation of motion of the crazyflie simulates the motor saturation.
#     :param F: desired total thrust, float
#     :param M: desired moments, 3 x 1 float vector
#     :return: thrust and moments after saturation
#     """
#     ## Equation of motion for X-configuration. From moments to rotor forces (power distribution)
#     #         ^ X
#     #    (4)  |      (1) [L, -L]
#     #   Y<-----
#     #    (3)         (2)
#     L = params.arm_length * sqrt(2) / 2
#     A = [[0.25, -.25 / L, -.25 / L],
#          [0.25, -.25 / L, .25 / L],
#          [0.25, .25 / L, .25 / L],
#          [0.25, .25 / L, -.25 / L]]
#
#     prop_thrusts = np.dot(A, [F, M[0], M[1]])  # Not using moment about Z-axis for limits
#     # Limit the force and moments due to actuator limits
#     # Motor saturation
#     prop_thrusts[prop_thrusts > params.maxF / 4] = params.maxF / 4
#     prop_thrusts[prop_thrusts < params.minF / 4] = params.minF / 4
#     prop_thrusts_clamped = prop_thrusts
#
#     # From prop forces to total moments
#     B = [[1, 1, 1, 1],
#          [-L, -L, L, L],
#          [-L, L, L, -L]]
#     # Total Thrust after saturation
#     nF = np.dot(B[0], prop_thrusts_clamped)
#
#     Mxy = np.dot(B[1:], prop_thrusts_clamped)
#     nM = [Mxy[0], Mxy[1], M[2]]
#
#     return nF, nM




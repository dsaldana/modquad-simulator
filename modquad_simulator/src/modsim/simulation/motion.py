from modsim import params
from modsim.util.quaternion import quaternion_to_matrix, matrix_to_quaternion, RPYtoRot_ZXY
import numpy as np
from modsim.util.state import state_to_quadrotor
from math import sqrt


def crazyflie_torquecontrol(F, M):
    """
    It receives a desired force and moment. The equation of motion of the crazyflie simulates the motor saturation.
    :param F: desired total thrust, float
    :param M: desired moments, 3 x 1 float vector
    :return: thrust and moments after saturation
    """
    ## Equation of motion for X-configuration. From moments to rotor forces (power distribution)
    #         ^ X
    #    (4)  |      (1) [L, -L]
    #   Y<-----
    #    (3)         (2)
    L = params.arm_length * sqrt(2) / 2
    A = [[0.25, -.25 / L, -.25 / L],
         [0.25, -.25 / L, .25 / L],
         [0.25, .25 / L, .25 / L],
         [0.25, .25 / L, -.25 / L]]

    prop_thrusts = np.dot(A, [F, M[0], M[1]])  # Not using moment about Z-axis for limits
    # Limit the force and moments due to actuator limits
    # Motor saturation
    prop_thrusts[prop_thrusts > params.maxF / 4] = params.maxF / 4
    prop_thrusts[prop_thrusts < params.minF / 4] = params.minF / 4
    prop_thrusts_clamped = prop_thrusts

    # From prop forces to total moments
    B = [[1, 1, 1, 1],
         [-L, -L, L, L],
         [-L, L, L, -L]]
    # Total Thrust after saturation
    nF = np.dot(B[0], prop_thrusts_clamped)

    Mxy = np.dot(B[1:], prop_thrusts_clamped)
    nM = [Mxy[0], Mxy[1], M[2]]

    return nF, nM


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
        rotor_forces[4 * mf[0] + mf[1]] *= 0.9

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


def state_derivative(state_vector, F, M, structure):
    """
    Calculate the derivative of the state vector. This is the function that needs to be integrated to know the next
    state of the robot.
    :param t: time
    :param state_vector: 13 x 1, state vector = [x, y, z, xd, yd, zd, qw, qx, qy, qz, p, q, r]
    :param F: thrust input for a single robot
    :param M: 3 x 1, moments output from controller (only used in simulation)
    :return: sdot: 13 x 1, derivative of state vector s
    """

    ## State of the quadrotor
    [xdot, ydot, zdot] = state_vector[3:6]  # Linear velocity
    quat = state_vector[6:10]  # orientation
    [qX, qY, qZ, qW] = quat
    bRw = quaternion_to_matrix(quat)
    wRb = bRw.T  # Orientation in matrix form
    omega = state_vector[10:]  # angular velocity
    [p, q, r] = omega
    # print r
    # Angular velocity
    K_quat = 2.  # this enforces the magnitude 1 constraint for the quaternion
    quat_error = 1 - (qW ** 2 + qX ** 2 + qY ** 2 + qZ ** 2)  # Quaternion error
    qdot = (-1. / 2) * np.dot(np.array([[0, -p, -q, -r],
                                        [p, 0, -r, q],
                                        [q, r, 0, -p],
                                        [r, -q, p, 0]]), [qW, qX, qY, qZ])
    qdot += K_quat * quat_error * np.array([qW, qX, qY, qZ])
    qdot = [qdot[1], qdot[2], qdot[3], qdot[0]]  # reorder the quaternion based on the ROS quaternion representation.

    # Angular acceleration
    # angular_acceleration = np.dot(params.invI, (M - np.cross(omega, np.dot(params.I, omega))))  # For a ingle robot
    angular_acceleration = np.dot(structure.inverse_inertia,
                                  (M - np.cross(omega, np.dot(structure.inertia_tensor, omega))))
    #
    # angular_acceleration [2]  = 0.  ## FIXME unknown acceleration in the z-axis
    # angular_acceleration[1] = 0.
    # Acceleration
    gravi = np.array([0, 0, structure.n * params.mass * params.grav])
    linear_acceleration = (np.dot(wRb, [0, 0, F]) - gravi) / params.mass

    ## Assemble the derivative of the state
    sdot = np.zeros(13)
    sdot[:3] = [xdot, ydot, zdot]  # linear velocity
    sdot[3:6] = linear_acceleration  # linear acceleration
    sdot[6:10] = qdot  # angular velocity as a quaternion
    sdot[10:] = angular_acceleration  # angular acceleration

    return sdot


# def control_output(s, desired_state, control_fun):
#     """
#     Computes the control output for a given state.
#
#     :param t: time
#     :param s: 13 x 1, state vector = [x, y, z, xd, yd, zd, qw, qx, qy, qz, p, q, r]
#     :param control_fun: function handle of your controller
#     :param trajhandle: function handle of your trajectory generator
#     :return: sdot: 13 x 1, derivative of state vector s
#     """
#     [des_pos, des_vel, des_acc, des_yaw, des_yawdot] = desired_state
#
#     # convert state to quad stuct for control
#     quadrotor = state_to_quadrotor(s)
#
#     # The desired_state is set in the trajectory generator
#     quadrotor.pos_des = des_pos
#     quadrotor.vel_des = des_vel
#     quadrotor.acc_des = des_acc
#     quadrotor.yaw_des = des_yaw
#     quadrotor.yawdot_des = des_yawdot
#
#     # get control outputs
#     # [F, M, trpy, drpy] = control_fun(quadrotor)
#     #
#     # return F, M
#
#     trpy = control_fun(quadrotor)
#
#     return trpy
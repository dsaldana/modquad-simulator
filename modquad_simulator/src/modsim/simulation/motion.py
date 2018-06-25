from modsim import params
from modsim.util.quaternion import quaternion_to_matrix, matrix_to_quaternion, RPYtoRot_ZXY
import numpy as np
from modsim.util.state import stateToQd
from math import sqrt


def crazyflie_motion_model(F, M):
    ## Equations of motion
    # X-configuration.
    L = params.arm_length * sqrt(2) / 2
    A = [[0.25, -.25 / L, -.25 / L],
         [0.25, -.25 / L, .25 / L],
         [0.25, .25 / L, .25 / L],
         [0.25, .25 / L, -.25 / L]]

    prop_thrusts = np.dot(A, [F, M[0], M[1]])  # Not using moment about Z-axis for limits
    # Limit the force and moments due to actuator limits
    # Motor saturation
    # FIXME: we can simulate the pwm saturation
    prop_thrusts[prop_thrusts > params.maxF / 4] = params.maxF / 4
    prop_thrusts[prop_thrusts < params.minF / 4] = params.minF / 4
    prop_thrusts_clamped = prop_thrusts

    B = [[1, 1, 1, 1],
         [-L, -L, L, L],
         [-L, L, L, -L]]
    # Force after saturation
    nF = np.dot(B[0], prop_thrusts_clamped)

    Mxy = np.dot(B[1:], prop_thrusts_clamped)
    nM = [Mxy[0], Mxy[1], M[2]]

    return nF, nM


def state_derivative(s, F, M):
    """
    Calculate the derivative of the state vector.
    :param t: time
    :param s: 13 x 1, state vector = [x, y, z, xd, yd, zd, qw, qx, qy, qz, p, q, r]
    :param F: thrust output from controller (only used in simulation)
    :param M: 3 x 1, moments output from controller (only used in simulation)
    :return: sdot: 13 x 1, derivative of state vector s
    """

    F, M = crazyflie_motion_model(F, M)

    [xdot, ydot, zdot] = s[3:6]
    quat = s[6:10]
    [qX, qY, qZ, qW] = quat
    omega = s[10:]
    [p, q, r] = omega

    bRw = quaternion_to_matrix(quat)
    wRb = bRw.T

    # Acceleration
    gravi = np.array([0, 0, params.mass * params.grav])
    accel = (np.dot(wRb, [0, 0, F]) - gravi) / params.mass

    # Angular velocity
    K_quat = 2  # this enforces the magnitude 1 constraint for the quaternion
    quaterror = 1 - (qW ** 2 + qX ** 2 + qY ** 2 + qZ ** 2)
    qdot = (-1 / 2) * np.dot(np.array([[0, -p, -q, -r],
                                       [p, 0, -r, q],
                                       [q, r, 0, -p],
                                       [r, -q, p, 0]]), [qW, qX, qY, qZ])
    qdot += K_quat * quaterror * np.array([qW, qX, qY, qZ])

    #
    qdot = [qdot[1], qdot[2], qdot[3], qdot[0]]
    # Angular acceleration
    pqrdot = np.dot(params.invI, (M - np.cross(omega, np.dot(params.I, omega))))

    # Assemble sdot
    sdot = np.zeros(13)
    sdot[:3] = [xdot, ydot, zdot]
    sdot[3:6] = accel
    sdot[6:10] = qdot
    sdot[10:] = pqrdot

    return sdot


def control_output(t, s, desired_state, control_fun):
    """
    Computes the control output for a given state.
    
    :param t: time
    :param s: 13 x 1, state vector = [x, y, z, xd, yd, zd, qw, qx, qy, qz, p, q, r]
    :param control_fun: function handle of your controller
    :param trajhandle: function handle of your trajectory generator
    :return: sdot: 13 x 1, derivative of state vector s
    """
    [des_pos, des_vel, des_acc, des_yaw, des_yawdot] = desired_state

    # convert state to quad stuct for control
    qd = stateToQd(s)

    # The desired_state is set in the trajectory generator
    qd.pos_des = des_pos
    qd.vel_des = des_vel
    qd.acc_des = des_acc
    qd.yaw_des = des_yaw
    qd.yawdot_des = des_yawdot

    # get control outputs
    [F, M, trpy, drpy] = control_fun(qd, t)

    return F, M

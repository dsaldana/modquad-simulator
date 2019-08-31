import math

from modsim.util.state import state_to_quadrotor
import numpy as np

# Old error
accumulated_error = np.array([0., 0., 0.])


def attitude_controller(control_in, x):
    """
    Attitude controller for crazyflie, receiving pwm as input.
    the output are forces and moments. F_newtons in Newtons
    :type control_in: tuple defined as (F_newtons, roll_des, pitch_des, yaw_des)
    :param x:
    :return:
    """
    global accumulated_error
    F_newtons = control_in[0]
    roll_des = control_in[1]
    pitch_des = control_in[2]
    yaw_des = control_in[3]

    ### Moments
    # Quaternion to angles
    quad_state = state_to_quadrotor(x)

    kpx, kdx, kix = 1.43e-5 * 250, 1.43e-5 * 60, .0002

    e = [math.radians(roll_des) - quad_state.euler[0],
         math.radians(pitch_des) - quad_state.euler[1],
         math.radians(yaw_des) - quad_state.euler[2]]

    accumulated_error += e
    #print(accumulated_error[0])

    Mx = kpx * e[0] + kdx * (0 - quad_state.omega[0]) + kix * accumulated_error[0]
    My = kpx * e[1] + kdx * (0 - quad_state.omega[1]) + kix * accumulated_error[1]
    #print(F_newtons, Mx, My)
    #print('---')
    return F_newtons, [Mx, My, 0]

import math

from modsim.util.state import state_to_quadrotor

# Old error
old_e = [0, 0, 0]


def attitude_controller((F_newtons, roll_des, pitch_des, yaw_des), x):
    """
    Attitude controller for crazyflie, receiving pwm as input.
    the output are forces and moments.
    :type F_newtons: force in newtons.
    :param x:
    :return:
    """
    global old_e

    ### Moments
    # Quaternion to angles
    quad_state = state_to_quadrotor(x)

    kpx, kdx = 1.43e-5 * 50, 1.43e-5 * 12

    e = [math.radians(roll_des) - quad_state.euler[0],
         math.radians(pitch_des) - quad_state.euler[1],
         math.radians(yaw_des) - quad_state.euler[2]]

    Mx = kpx * e[0] + kdx * (0 - quad_state.omega[0])
    My = kpx * e[1] + kdx * (0 - quad_state.omega[1])
    return F_newtons, [Mx, My, 0]

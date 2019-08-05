import math

from modsim.util.state import stateToQd

# Old error
old_e = [0, 0, 0]


def attitude_controller((thrust_pwm, roll_des, pitch_des, yaw_des), x):
    """
    Attitude controller for crazyflie, receiving pwm as input.
    the output are forces and moments.
    :param x:
    :return:
    """
    global old_e

    ### Force
    c1, c2, c3 = -0.6709, 0.1932, 13.0652
    F_g = ((thrust_pwm / 60000. - c1) / c2) ** 2 - c3  # Force in grams
    F = 9.81 * F_g / 1000.  # Force in Newtons

    ### Moments
    # Quaternion to angles
    quad_state = stateToQd(x)

    kpx, kdx = 1.43e-5 * 50, 1.43e-5 * 12

    e = [math.radians(roll_des) - quad_state.euler[0],
         math.radians(pitch_des) - quad_state.euler[1],
         math.radians(yaw_des) - quad_state.euler[2]]

    Mx = kpx * e[0] + kdx * (0 - quad_state.omega[0])
    My = kpx * e[1] + kdx * (0 - quad_state.omega[1])
    return F, [Mx, My, 0]

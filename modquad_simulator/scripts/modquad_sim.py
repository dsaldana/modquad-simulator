#!/usr/bin/env python


import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from scipy.integrate import ode

from modsim.attitude import attitude_controller
# from modsim.plot.drawer_vispy import Drawer
from modsim.simulation.motion import state_derivative
from modsim.util.comm import publish_odom, publish_transform_stamped
from modsim.util.state import init_state, stateToQd
from modquad_simulator.srv import Dislocation, DislocationResponse

# Control Input
thrust_pwm, roll, pitch, yaw = 0, 0, 0, 0


# Control input callback
def control_input_listenener(twist_msg):
    global thrust_pwm, roll, pitch, yaw
    # For more info, check:
    # https://github.com/whoenig/crazyflie_ros
    roll = twist_msg.linear.y
    pitch = twist_msg.linear.x
    yaw = twist_msg.angular.z
    thrust_pwm = twist_msg.linear.z


dislocation_srv = (0., 0.)


def dislocate(disloc_msg):
    global dislocation_srv
    dislocation_srv = (disloc_msg.x, disloc_msg.y)
    return DislocationResponse()  # Return nothing


def simulate():
    global dislocation_srv
    rospy.init_node('modrotor_simulator', anonymous=True)
    robot_id = rospy.get_param('~robot_id', 'modquad01')

    init_x = rospy.get_param('~init_x', 0.)
    init_y = rospy.get_param('~init_y', 0.)
    init_z = rospy.get_param('~init_z', 0.)

    odom_topic = rospy.get_param('~odom_topic', '/odom')  # '/odom2'
    # cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')  # '/cmd_vel2'

    # viewer = rospy.get_param('~viewer', False)

    # service for dislocate the robot
    rospy.Service('dislocate_robot', Dislocation, dislocate)

    # Subscribe to control input
    rospy.Subscriber('/' + robot_id + '/cmd_vel', Twist, control_input_listenener)

    # Odom publisher
    odom_pub = rospy.Publisher('/' + robot_id + odom_topic, Odometry, queue_size=0)
    # TF publisher
    tf_broadcaster = tf2_ros.TransformBroadcaster()


    ########### Simulator ##############
    loc = [init_x, init_y, init_z]
    x = init_state(loc, 0)

    freq = 100  # 100hz
    rate = rospy.Rate(freq)
    while not rospy.is_shutdown():
        rate.sleep()

        # Dislocate based on request
        x[0] += dislocation_srv[0]
        x[1] += dislocation_srv[1]
        dislocation_srv = (0., 0.)

        # Publish odometry
        publish_odom(x, odom_pub)
        publish_transform_stamped(robot_id, x, tf_broadcaster)

        # Control output
        F, M = attitude_controller((thrust_pwm, roll, pitch, yaw), x)

        # Derivative of the robot dynamics
        f_dot = lambda t1, s: state_derivative(s, F, M)

        # Solve the differential equation of motion
        r = ode(f_dot).set_integrator('dopri5')
        r.set_initial_value(x, 0)
        r.integrate(r.t + 1. / freq, step=True)
        if not r.successful():
            return
        x = r.y

        # Simulate floor
        if x[2] < 0:
            x[2] = 0.
            # Velocity towards the floor
            if x[5] < 0:
                x[5] = 0.


if __name__ == '__main__':
    simulate()

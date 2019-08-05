#!/usr/bin/env python


import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from numpy import copy
from scipy.integrate import ode

from modsim import params
from modsim.attitude import attitude_controller
# from modsim.plot.drawer_vispy import Drawer
from modsim.controller import control_handle
from modsim.datatype.structure import Structure
from modsim.simulation.motion import state_derivative, control_output
from modsim.trajectory import circular_trajectory
from modsim.util.comm import publish_odom, publish_transform_stamped, publish_odom_relative, \
    publish_transform_stamped_relative
from modsim.util.state import init_state, stateToQd
from modquad_simulator.srv import Dislocation, DislocationResponse

# Control Input
thrust_pwm, roll, pitch, yaw = 0, 0, 0, 0


# Control input callback
def control_input_listener(twist_msg):
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


def publish_structure_odometry(structure, x, odom_publishers, tf_broadcaster):
    ids, xx, yy = structure.ids, structure.xx, structure.yy

    # publish main robot
    main_id = ids[0]
    publish_odom(x, odom_publishers[main_id])
    publish_transform_stamped(main_id, x, tf_broadcaster)

    # show the other robots
    for robot_id, structure_x, structure_y in zip(ids, xx, yy)[1:]:
        publish_odom_relative(structure_x, structure_y, robot_id, main_id, odom_publishers[robot_id])
        publish_transform_stamped_relative(robot_id, main_id, structure_x, structure_y, tf_broadcaster)


def simulate():
    global dislocation_srv
    rospy.init_node('modrotor_simulator', anonymous=True)
    robot_id = rospy.get_param('~robot_id', 'modquad01')

    init_x = rospy.get_param('~init_x', 0.)
    init_y = rospy.get_param('~init_y', 0.)
    init_z = rospy.get_param('~init_z', 0.)

    odom_topic = rospy.get_param('~odom_topic', '/odom')  # '/odom2'
    # cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')  # '/cmd_vel2'

    # service for dislocate the robot
    rospy.Service('dislocate_robot', Dislocation, dislocate)

    # TODO read structure and create a service to change it.
    # structure = Structure(ids=['modquad01', 'modquad02'], xx=[0, -params.cage_width], yy=[0, 0])
    structure = Structure()

    # Subscribe to control input
    rospy.Subscriber('/' + robot_id + '/cmd_vel', Twist, control_input_listener)

    # Odom publisher
    odom_publishers = {id_robot: rospy.Publisher('/' + id_robot + odom_topic, Odometry, queue_size=0) for id_robot in
                       structure.ids}
    # TF publisher
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    ########### Simulator ##############
    loc = [init_x, init_y, init_z]
    state_vector = init_state(loc, 0)

    freq = 100  # 100hz
    rate = rospy.Rate(freq)
    t = 0
    while not rospy.is_shutdown():
        rate.sleep()
        t += 1. / freq

        ## Dislocate based on request
        state_vector[0] += dislocation_srv[0]
        state_vector[1] += dislocation_srv[1]
        dislocation_srv = (0., 0.)

        ## Publish odometry
        publish_structure_odometry(structure, state_vector, odom_publishers, tf_broadcaster)

        ##### Trajectory
        desired_state = circular_trajectory(t % 10, 10)
        # Position controller
        [F, M] = control_output(t, state_vector, desired_state, control_handle)
        # if t > 30:
        #     F, M = 0, [0, 0, 0]

        ##### Control output based on crazyflie input
        # F, M = attitude_controller((thrust_pwm, roll, pitch, yaw), state_vector)

        ## Derivative of the robot dynamics
        f_dot = lambda t1, s: state_derivative(s, F, M, structure)

        # Solve the differential equation of motion
        r = ode(f_dot).set_integrator('dopri5')
        r.set_initial_value(state_vector, 0)
        r.integrate(r.t + 1. / freq, step=True)
        if not r.successful():
            return
        state_vector = r.y

        # Simulate floor
        if state_vector[2] < 0:
            state_vector[2] = 0.
            # Velocity towards the floor
            if state_vector[5] < 0:
                state_vector[5] = 0.


if __name__ == '__main__':
    simulate()

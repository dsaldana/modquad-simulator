#!/usr/bin/env python


import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from numpy import copy
from modsim.controller import control_handle
from modsim.trajectory import circular_trajectory, simple_waypt_trajectory, \
        min_snap_trajectory

from modsim.simulation.motion import control_output, modquad_torque_control

from modsim import params
from modsim.attitude import attitude_controller
# from modsim.plot.drawer_vispy import Drawer

from modsim.datatype.structure import Structure


from modsim.util.comm import publish_odom, publish_transform_stamped, publish_odom_relative, \
    publish_transform_stamped_relative
from modsim.util.state import init_state, stateToQd
from modquad_simulator.srv import Dislocation, DislocationResponse
from modsim.simulation.ode_integrator import simulation_step


# Control Input
thrust_pwm, roll, pitch, yaw = 0, 0, 0, 0

dislocation_srv = (0., 0.)

# Control input callback
def control_input_listener(twist_msg):
    global thrust_pwm, roll, pitch, yaw
    # For more info, check:
    # https://github.com/whoenig/crazyflie_ros
    roll = twist_msg.linear.y
    pitch = twist_msg.linear.x
    yaw = twist_msg.angular.z
    thrust_pwm = twist_msg.linear.z


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
    demo_trajectory = rospy.get_param('~demo_trajectory', True)

    odom_topic = rospy.get_param('~odom_topic', '/odom')  # '/odom2'
    # cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')  # '/cmd_vel2'

    # service for dislocate the robot
    rospy.Service('dislocate_robot', Dislocation, dislocate)

    # TODO read structure and create a service to change it.
    # structure = Structure(ids=['modquad01', 'modquad02'], xx=[0, -params.cage_width], yy=[0, 0])
    structure = Structure(ids=[robot_id], xx=[0], yy=[0])

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
    waypts = np.array([ [0,0,0],
                        [0,0.5,0],
                        [0.5,0.5,0],
                        [0.5,0,0],
                        [0,0,0]])
                        

    traj_vars = min_snap_trajectory(0, 10, None, waypts)
    while not rospy.is_shutdown():
        rate.sleep()
        t += 1. / freq

        ## Dislocate based on request
        state_vector[0] += dislocation_srv[0]
        state_vector[1] += dislocation_srv[1]
        dislocation_srv = (0., 0.)

        # Publish odometry
        publish_structure_odometry(structure, state_vector, odom_publishers, tf_broadcaster)

        # Control output based on crazyflie input
        F, M = attitude_controller((thrust_pwm, roll, pitch, yaw), state_vector)

        
        if demo_trajectory:
            F, M = control_output(t, state_vector, 
                    min_snap_trajectory(t % 10, 30, traj_vars), control_handle)
            #F, M = control_output(t, state_vector, 
            #        simple_waypt_trajectory(waypts, t % 10, 30), control_handle)
            #F, M = control_output(t, state_vector, 
            #        circular_trajectory(t % 10, 10), control_handle)

        # Control of Moments and thrust
        F_structure, M_structure, rotor_forces = modquad_torque_control(F, M, structure)

        # Simulate
        state_vector = simulation_step(structure, state_vector, 
                F_structure, M_structure, 1./freq)

if __name__ == '__main__':
    simulate()

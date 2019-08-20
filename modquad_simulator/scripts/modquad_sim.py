#!/usr/bin/env python


import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from numpy import copy
from modsim.controller import position_controller, modquad_torque_control
from modsim.trajectory import circular_trajectory, simple_waypt_trajectory, \
    min_snap_trajectory

from modsim import params
from modsim.attitude import attitude_controller
# from modsim.plot.drawer_vispy import Drawer

from modsim.datatype.structure import Structure

from modsim.util.comm import publish_odom, publish_transform_stamped, publish_odom_relative, \
    publish_transform_stamped_relative
from modsim.util.state import init_state, state_to_quadrotor
from modquad_simulator.srv import Dislocation, DislocationResponse
from modsim.simulation.ode_integrator import simulation_step

from modquad_sched_interface.interface import convert_modset_to_struc
import modquad_sched_interface.waypt_gen as waypt_gen
import modquad_sched_interface.structure_gen as structure_gen


# Control Input
thrust_newtons, roll, pitch, yaw = 0., 0., 0., 0.

dislocation_srv = (0., 0.)


# Control input callback
def control_input_listener(twist_msg):
    global thrust_newtons, roll, pitch, yaw
    # For more info, check:
    # https://github.com/whoenig/crazyflie_ros
    roll = twist_msg.linear.y
    pitch = twist_msg.linear.x
    yaw = twist_msg.angular.z
    thrust_pwm = twist_msg.linear.z

    c1, c2, c3 = -0.6709, 0.1932, 13.0652
    F_g = ((thrust_pwm / 60000. - c1) / c2) ** 2 - c3  # Force in grams
    if F_g<0:
        F_g = 0

    thrust_newtons = 9.81 * F_g / 1000.  # Force in Newtons


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
        publish_odom_relative(structure_x - xx[0], structure_y - yy[0], robot_id, main_id, odom_publishers[robot_id])
        publish_transform_stamped_relative(robot_id, main_id, structure_x - xx[0], structure_y - yy[0], tf_broadcaster)

def simulate(structure, trajectory_function, t_step=0.01, speed=1, loc=[1., .0, .0], 
        waypts=None, figind=1, filesuffix=""):
    global dislocation_srv, thrust_newtons, roll, pitch, yaw
    rospy.init_node('modrotor_simulator', anonymous=True)
    robot_id1 = rospy.get_param('~robot_id', 'modquad01')
    rids = [robot_id1]#, robot_id2] #[robot_id1, robot_id2, robot_id3, robot_id4]

    init_x = rospy.get_param('~init_x', 0.)
    init_y = rospy.get_param('~init_y', 0.)
    init_z = rospy.get_param('~init_z', 0.)
    demo_trajectory = rospy.get_param('~demo_trajectory', True)

    odom_topic = rospy.get_param('~odom_topic', '/odom')  # '/odom2'
    # cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')  # '/cmd_vel2'

    # service for dislocate the robot
    rospy.Service('dislocate_robot', Dislocation, dislocate)

    # TODO read structure and create a service to change it.
    
    #structure4 = Structure(ids=['modquad01', 'modquad02'],
    #                       xx=[0, params.cage_width, 0, params.cage_width], yy=[0, 0, params.cage_width, params.cage_width],
    #structure4 = Structure(ids=['modquad01', 'modquad02', 'modquad03', 'modquad04'],
    #                       xx=[0, params.cage_width, 0, params.cage_width], yy=[0, 0, params.cage_width, params.cage_width],
    #                       motor_failure=[])
    #structure4fail = Structure(ids=['modquad01', 'modquad02'],
    #                       xx=[0, params.cage_width, 0, params.cage_width],
    #                       yy=[0, 0, params.cage_width, params.cage_width],
    #                       motor_failure=[(1, 0)])
    #structure1 = Structure(ids=[robot_id], xx=[0], yy=[0])
    #structure = structure4fail

    #structure = structure4 #structure_gen.zero(3, 2), 
    #waypts = waypt_gen.line([0,0,0], [3,3,3])
    #waypts = waypt_gen.spiral(3, 3, 3, 2)
    #speed = 1.25
    #trajectory_function = min_snap_trajectory
    traj_vars = trajectory_function(0, speed, None, waypts)
    tmax = traj_vars.total_dist / speed

    # Subscribe to control input
    [rospy.Subscriber('/' + robot_id + '/cmd_vel', Twist, control_input_listener) for robot_id in rids]

    print(structure.ids)
    print(structure.xx)
    print(structure.yy)
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

        # Publish odometry
        publish_structure_odometry(structure, state_vector, odom_publishers, tf_broadcaster)

        if demo_trajectory:
            # F, M = control_output( state_vector,
            #         min_snap_trajectory(t % 10, 30, traj_vars), control_handle)
            # F, M = control_output( state_vector,
            #        simple_waypt_trajectory(waypts, t % 10, 30), control_handle)
            # F, M = control_output( state_vector,
            #                       circular_trajectory(t % 10, 10), control_handle)

            # Overwrite the control input with the demo trajectory
            [thrust_newtons, roll, pitch, yaw] = position_controller(state_vector, trajectory_function(t, speed, traj_vars))

        # Control output based on crazyflie input
        F_single, M_single = attitude_controller((thrust_newtons, roll, pitch, yaw), state_vector)

        # Control of Moments and thrust
        F_structure, M_structure, rotor_forces = modquad_torque_control(F_single, M_single, structure, motor_sat=True)

        # Simulate
        state_vector = simulation_step(structure, state_vector, F_structure, M_structure, 1. / freq)
        # state_vector[-1] = 0.01-state_vector[-1]


#if __name__ == '__main__':
#    simulate()

def test_shape_with_waypts(mset, wayptset, speed=1, test_id=""):
    from modquad_sched_interface.interface import convert_modset_to_struc
    from compiled_scheduler.modset import modset
    from scheduler.gsolver import gsolve
    trajectory_function = min_snap_trajectory
    traj_vars = trajectory_function(0, speed, None, wayptset)

    # Run Set 1
    gsolve(mset, waypts=traj_vars.waypts)
    #mset.fault_rotor(0,0)
    #mset.fault_rotor(0,1)
    #mset.fault_rotor(0,2)
    #mset.fault_rotor(2,3)
    struc1 = convert_modset_to_struc(mset)
    print(struc1.xx)
    print(struc1.yy)
    #import sys
    #sys.exit(0)
    simulate(struc1, trajectory_function, waypts=wayptset, loc=[0,0,0], figind=1, speed=speed, filesuffix="{}_noreform".format(test_id))

    #tmp = copy.deepcopy(mset)
    ##tmp.fault_rotor(2, 3)
    #gsolve(tmp, waypts=traj_vars.waypts)
    #struc2 = convert_modset_to_struc(tmp)
    #forces2, pos_err2 = simulate(struc2, trajectory_function, waypts=wayptset, loc=[0,0,0], figind=1, speed=speed, filesuffix="{}_reform".format(test_id))
    #return [forces1, forces2, pos_err1, pos_err2]
    return None

if __name__ == '__main__':
    import modquad_sched_interface.waypt_gen as waypt_gen
    import modquad_sched_interface.structure_gen as structure_gen
    import sys
    print("starting simulation")
    #mset = structure_gen.square(2)
    results = test_shape_with_waypts(
                       #structure_gen.fwd_ushape(3, 2),
                       structure_gen.zero(3, 3), 
                       #structure_gen.square(2), 
                       #waypt_gen.line([0,0,0], [i,i,i]), 
                       #waypt_gen.rect(10,10),
                       #waypt_gen.zigzag_xy(10,5),
                       waypt_gen.spiral(3,3,3,2),
                       speed=0.25, test_id="motorsat2_rect10x10_4x4full")

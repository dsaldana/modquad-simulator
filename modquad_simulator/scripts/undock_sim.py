#!/usr/bin/env python
import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from numpy import copy
import networkx as nx
import sys

from modsim.trajectory import circular_trajectory, simple_waypt_trajectory, min_snap_trajectory

from modsim import params

from modsim.datatype.structure import Structure
from modsim.datatype.structure_manager import StructureManager
from dockmgr.datatype.disassembly_manager import DisassemblyManager
#from dockmgr.datatype.assembly_manager import AssemblyManager

from modsim.util.comm import publish_odom, publish_transform_stamped, publish_odom_relative, \
    publish_transform_stamped_relative
from modsim.util.state import init_state, state_to_quadrotor
from modsim.util.undocking import gen_strucs_from_split, split_srv_input_format
from modquad_simulator.srv import Dislocation, DislocationResponse, SplitStructure, SplitStructureResponse
from modsim.simulation.ode_integrator import simulation_step

from modquad_sched_interface.interface import convert_modset_to_struc, convert_struc_to_mat
import modquad_sched_interface.waypt_gen as waypt_gen
import modquad_sched_interface.structure_gen as structure_gen

from scheduler.gsolver import gsolve
from scheduler.reconfigure import reconfigure


## Control Input
#thrust_newtons, roll, pitch, yaw = 0., 0., 0., 0.
#
dislocation_srv = (0., 0.)
#
## Control input callback
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


#def publish_structure_odometry(structure, x, odom_publishers, tf_broadcaster):
#    ids, xx, yy = structure.ids, structure.xx, structure.yy
#
#    # publish main robot
#    main_id = ids[0]
#    publish_odom(x, odom_publishers[main_id])
#    publish_transform_stamped(main_id, x, tf_broadcaster)
#
#    # show the other robots
#    for robot_id, structure_x, structure_y in zip(ids, xx, yy)[1:]:
#        publish_odom_relative(structure_x - xx[0], structure_y - yy[0], robot_id, main_id, odom_publishers[robot_id])
#        publish_transform_stamped_relative(robot_id, main_id, structure_x - xx[0], structure_y - yy[0], tf_broadcaster)

def simulate(struc, newstruc, reconf_map, trajectory_function, t_step=0.01, speed=1, loc=[1., .0, .0], 
        waypts=None, figind=1, filesuffix="", split_dim=0, breakline=1, split_ind=0):
    global dislocation_srv, thrust_newtons, roll, pitch, yaw
    rospy.init_node('modrotor_simulator', anonymous=True)
    robot_id1 = rospy.get_param('~robot_id', 'modquad01')
    rids = [robot_id1]

    init_x = rospy.get_param('~init_x', 0.)
    init_y = rospy.get_param('~init_y', 0.)
    init_z = rospy.get_param('~init_z', 0.)
    demo_trajectory = rospy.get_param('~demo_trajectory', True)

    rospy.set_param('opmode', 'normal')
    rospy.set_param('structure_speed', speed)

    odom_topic = rospy.get_param('~odom_topic', '/odom')  # '/odom2'
    # cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')  # '/cmd_vel2'

    # service for dislocate the robot
    rospy.Service('dislocate_robot', Dislocation, dislocate)

    # TODO read structure and create a service to change it.

    # Min snap trajectory persistent vars init
    # This step done in the setup already
    traj_vars = trajectory_function(0, speed, None, waypts)
    strucs = [struc]
    trajs  = [traj_vars]

    # Time based on avg desired speed (actual speed *not* constant)
    tmax = traj_vars.total_dist / speed 

    # Subscribe to control input
    [rospy.Subscriber('/' + robot_id + '/cmd_vel', Twist, control_input_listener) for robot_id in rids]

    # Odom publisher
    odom_publishers = {id_robot: rospy.Publisher('/' + id_robot + odom_topic, Odometry, queue_size=0) for id_robot in
                       struc.ids}
    # TF publisher
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    ########### Simulator ##############
    loc = [init_x, init_y, init_z]
    state_vector = init_state(loc, 0)
    state_vecs = [state_vector]

    undocked = False
    freq = 100  # 100hz
    rate = rospy.Rate(freq)
    t = 0

    struc_mgr = StructureManager(strucs, state_vecs, trajs, freq)
    disassembler = None
    
    while not rospy.is_shutdown():
        rate.sleep()
        t += 1. / freq

        if rospy.get_param('opmode') == 'disassemble':
            disassembler.take_step(struc_mgr, t)
        # Assuming adherence to trajectory that is already loaded in
        # StructureManager handles doing the actual physics of the simulation for
        # all structures, and hence for all individual modules
        struc_mgr.control_step(t, trajectory_function, speed, 
                odom_publishers, tf_broadcaster)

        if t > 3 and not undocked: # Split the structure
            #structure = struc_mgr.get_structure_by_ind(0)
            index = 0
            structure, _, traj_vars = struc_mgr.get_data_by_ind(index)
            rospy.wait_for_service('SplitStructure')
            np.set_printoptions(precision=3)
            np.set_printoptions(suppress=True)
            np.set_printoptions(threshold=sys.maxsize)
            print("Undocking commencing")
            #print(convert_struc_to_mat(struc.ids, struc.xx, struc.yy))
            #print(struc.ids)
            #print(struc.xx)
            #print(struc.yy)
            disassembler = DisassemblyManager(structure, reconf_map, t, struc_mgr, trajectory_function)
            #struc_mgr.disassemble(reconf_map)
            #try:
            #    # Split the structure physically
            #    split = rospy.ServiceProxy("SplitStructure", SplitStructure)
            #    inp = split_srv_input_format(structure, split_dim, breakline, split_ind)
            #    ret = split(inp[0], inp[1], inp[2], inp[3], inp[4], inp[5], inp[6], inp[7])

            #    # Generate the new structures post-split (i.e. generate the actual objects)
            #    strucs = gen_strucs_from_split(ret)

            #    # Generate trajectories for new structures
            #    traj_vars1 = trajectory_function(0, speed, None, waypt_gen.line([state_vecs[0][0]   , state_vecs[0][1]   , state_vecs[0][2]   ], 
            #                                                                     [state_vecs[0][0]-15, state_vecs[0][1]-15, state_vecs[0][2]+15] ))
            #    traj_vars2 = trajectory_function(0, speed, None, waypt_gen.spiral(5,5,5,2, start_pt=[state_vecs[0][0], state_vecs[0][1], state_vecs[0][2]]))
            #    trajs = [traj_vars1, traj_vars2]
            #    state_vecs = [state_vecs[0], state_vecs[0]]

            #    # Update the structure manager
            #    struc_mgr.split_struc(index, strucs, trajs, state_vecs)

            #    # Set sim params
            #except rospy.ServiceException, e:
            #    print("SplitStructure service call failed: {}".format(e))
            #    import sys
            #    sys.exit(0)
            t = 0
            undocked = True

def test_undock_along_path(mset1, wayptset, speed=1, test_id="", split_dim=0, breakline=1, split_ind=0):
    # Import here in case want to run w/o mqscheduler package
    from modquad_sched_interface.interface import convert_modset_to_struc
    from compiled_scheduler.modset import modset

    # Setup
    trajectory_function = min_snap_trajectory
    traj_vars = trajectory_function(0, speed, None, wayptset)

    gsolve(mset1, waypts=traj_vars.waypts, speed=speed)

    mset1.fault_rotor(4, 0)
    struc1 = convert_modset_to_struc(mset1)
    mset2 = modset(mset1.num_mod, np.copy(mset1.struc), mset1, mset1.mod_ids)
    gsolve(mset2, waypts=traj_vars.waypts, speed=speed)
    struc2 = convert_modset_to_struc(mset2)

    #print('Print modules mset1')
    #for mod in mset1.mods:
    #    print(mod)
    #    print("\t{}".format(mod.hashstring))
    #print('Print modules mset2')
    #for mod in mset2.mods:
    #    print(mod)
    #    print("\t{}".format(mod.hashstring))
    #print('-----')


    [cost, reconf_map] = reconfigure(mset1, mset2, waypts=traj_vars.waypts, speed=speed)
    print("Reconfigure this structure:")
    print(mset1.pi)
    print("To this structure:")
    print(mset2.pi)
    print('=======================')
    #print('((((')
    #print(mset1.pi)
    #m1s = convert_modset_to_struc(mset1)
    #print(m1s.ids)
    #print(m1s.xx)
    #print(m1s.yy)
    #print(convert_struc_to_mat(m1s.ids, m1s.xx, m1s.yy))
    #print('----')
    #print(mset2.pi)
    #m2s = convert_modset_to_struc(mset2)
    #print(m2s.ids)
    #print(m2s.xx)
    #print(m2s.yy)
    #print(convert_struc_to_mat(m2s.ids, m2s.xx, m2s.yy))
    #print('))))')

    simulate(struc1, struc2, reconf_map, trajectory_function, waypts=wayptset, loc=[0,0,0], figind=1, speed=speed, filesuffix="{}_noreform".format(test_id))

    return None

if __name__ == '__main__':
    # Import here in case want to run w/o mqscheduler package
    import modquad_sched_interface.waypt_gen as waypt_gen
    import modquad_sched_interface.structure_gen as structure_gen
    import sys
    print("Starting Undocking Simulation")
    results = test_undock_along_path(
                       #structure_gen.fwd_ushape(3, 2),
                       structure_gen.zero(3, 3), 
                       #structure_gen.square(3), 
                       waypt_gen.line([0,0,0], [15,15,1]), 
                       #waypt_gen.rect(10,10),
                       #waypt_gen.zigzag_xy(10,5),
                       #waypt_gen.spiral(3,3,3,2),
                       speed=0.55, test_id="motorsat2_rect10x10_4x4full", split_dim=0, breakline=1, split_ind=0)

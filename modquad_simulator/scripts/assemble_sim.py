"""
This simulation only tests docking.
You can change the structure type at the bottom, and nominally pass in
a trajectory. 

setup_and_test() will instantiate nine structures and invoke the IROS 2017 
algorithm to assemble the modquad modules into a desired structure
"""
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
thrust_newtons, roll, pitch, yaw = 0., 0., 0., 0.
num_mod = 9

dislocation_srv = (0., 0.)

# Control input callback
#def control_input_listener(twist_msg):
#    #global thrust_newtons, roll, pitch, yaw
#    # For more info, check:
#    # https://github.com/whoenig/crazyflie_ros
#    global struc_mgr
#    if struc_mgr is not None:
#        struc_mgr.control_input_listener(twist_msg)
#    #roll = twist_msg.linear.y
#    #pitch = twist_msg.linear.x
#    #yaw = twist_msg.angular.z
#    #thrust_pwm = twist_msg.linear.z
#
#    #c1, c2, c3 = -0.6709, 0.1932, 13.0652
#    #F_g = ((thrust_pwm / 60000. - c1) / c2) ** 2 - c3  # Force in grams
#    #if F_g<0:
#    #    F_g = 0
#
#    #thrust_newtons = 9.81 * F_g / 1000.  # Force in Newtons

#def dislocate(disloc_msg):
#    global dislocation_srv
#    dislocation_srv = (disloc_msg.x, disloc_msg.y)
#    return DislocationResponse()  # Return nothing

def simulate(struc_mgr, pi, trajectory_function):
    #global dislocation_srv, thrust_newtons, roll, pitch, yaw
    rospy.init_node('modrotor_simulator', anonymous=True)
    robot_id1 = rospy.get_param('~robot_id', 'modquad01')
    rids = [robot_id1]

    init_x = rospy.get_param('~init_x', 0.)
    init_y = rospy.get_param('~init_y', 0.)
    init_z = rospy.get_param('~init_z', 0.)
    demo_trajectory = rospy.get_param('~demo_trajectory', True)

    speed = rospy.get_param('structure_speed', 1.0)
    rospy.set_param('opmode', 'normal')
    rospy.set_param('rotor_map', 2) # So that modquad_torque_control knows which mapping to use

    odom_topic = rospy.get_param('~odom_topic', '/odom')  # '/odom2'
    # cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')  # '/cmd_vel2'

    # service for dislocate the robot
    #rospy.Service('dislocate_robot', Dislocation, dislocate)

    # Subscribe to control input, note callback is in structure manager
    # Commenting this makes no difference
    #[rospy.Subscriber('/' + robot_id + '/cmd_vel', Twist, control_input_listener) for robot_id in rids]

    # Odom publisher
    odom_publishers = {id_robot: 
        rospy.Publisher('/' + id_robot + odom_topic, Odometry, queue_size=0) 
        for struc in struc_mgr.strucs for id_robot in struc.ids}

    # TF publisher
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    ########### Simulator ##############

    # Location of first structure
    loc = [init_x, init_y, init_z]
    state_vector = init_state(loc, 0)

    # Time based on avg desired speed (actual speed *not* constant)
    tmax = 20.0
    overtime = 1.5

    # Params
    freq = 100   # 100hz
    rate = rospy.Rate(freq)
    rospy.set_param("freq", freq)
    t = 0

    # As a start, first get everyone to move so we see they are functional
    for s in struc_mgr.strucs:
        s.traj_vars = trajectory_function(0, speed, None, 
                waypt_gen.line(s.state_vector[:3], s.state_vector[:3] + 1))

    # Don't start with a assembler object
    assembler = AssemblyManager(t, pi, trajectory_function)
    assembler.find_assembly_tree(struc_mgr)
    docking = False
    ind = 0
    while not rospy.is_shutdown() and t < overtime * tmax:
        rate.sleep()
        t += 1. / freq

        opmode = rospy.get_param('opmode', 'normal')
        if opmode == 'disassemble':
            disassembler.take_step(struc_mgr, t)
        elif opmode == 'assemble':
            assembler.take_step(struc_mgr, t)

        # Assuming adherence to trajectory that is already loaded in
        # StructureManager handles doing the actual physics of the simulation for
        # all structures, and hence for all individual modules
        struc_mgr.control_step(t, trajectory_function, speed, 
                odom_publishers, tf_broadcaster)

        if t > 4.0 and not docking: # Start assembling
            #rospy.wait_for_service('SplitStructure')
            print("Parallelized undocking procedure triggered")
            docking = True

        ind += 1
    struc_mgr.make_plots()

def test_assembly(mset1, wayptset):
    # Import here in case want to run w/o mqscheduler package
    from modquad_sched_interface.interface import convert_modset_to_struc
    from compiled_scheduler.modset import modset

    global num_mod
    speed = rospy.get_param('structure_speed', 1.0)

    # Setup
    trajectory_function = min_snap_trajectory
    traj_vars = trajectory_function(0, speed, None, wayptset)
    gsolve(mset1, waypts=traj_vars.waypts, speed=speed)

    # Generate all nine structures as individual modules and make struc_mgr
    # Note that we don't set up the trajectory for these
    strucs = [Structure(['modquad{:02d}'.format(i+1)], xx=[0.0], yy=[0.0]) for i in range(num_mod)]
    for i,s in enumerate(strucs):
        s.state_vector = init_state([i+0.0,0.0,0.0], 0)
    struc_mgr = StructureManager(strucs)

    print("Assemble this structure (No modid = 0 -- that is empty space):")
    print(mset1.pi + 1)
    print('=======================')

    # 8. Run the simulation of the breakup and reassembly
    simulate(struc_mgr, mset1.pi, trajectory_function)

if __name__ == '__main__':
    print("Starting Assembly Simulation")
    rospy.set_param('structure_speed', 0.5)
    test_assembly(structure_gen.square(3), waypt_gen.line([0,0,0],[10,15,2]))

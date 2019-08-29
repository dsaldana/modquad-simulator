#!/usr/bin/env python
import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from numpy import copy
import networkx as nx

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
from modsim.util.undocking import gen_strucs_from_split, split_srv_input_format

from modquad_simulator.srv import Dislocation, DislocationResponse
from modquad_simulator.srv import SplitStructure, SplitStructureResponse

from modsim.simulation.ode_integrator import simulation_step

from dockmgr.datatype.ComponentManager import ComponentManager
from dockmgr.datatype.OdometryManager import OdometryManager

from modquad_sched_interface.interface import convert_modset_to_struc, convert_struc_to_mat
"""
Uses ComponentManager to manage the structures that are available in
whatever the current simulation happens to be
"""

thrust_newtons, roll, pitch, yaw = 0.0, 0.0, 0.0, 0.0

## Control input callback
#def control_input_listener(twist_msg):
#    global thrust_newtons, roll, pitch, yaw
#    # For more info, check:
#    # https://github.com/whoenig/crazyflie_ros
#    roll = twist_msg.linear.y
#    pitch = twist_msg.linear.x
#    yaw = twist_msg.angular.z
#    thrust_pwm = twist_msg.linear.z
#
#    c1, c2, c3 = -0.6709, 0.1932, 13.0652
#    F_g = ((thrust_pwm / 60000. - c1) / c2) ** 2 - c3  # Force in grams
#    if F_g<0:
#        F_g = 0
#
#    thrust_newtons = 9.81 * F_g / 1000.  # Force in Newtons
#
#
#def dislocate(disloc_msg):
#    global dislocation_srv
#    dislocation_srv = (disloc_msg.x, disloc_msg.y)
#    return DislocationResponse()  # Return nothing


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

class StructureManager:
    def __init__(self, struclist, state_vecs, traj_vars, freq):
        self.strucs = struclist # init based on those passed in
        self.state_vecs = state_vecs
        self.trajs = traj_vars
        self.demo_trajectory = True#rospy.get_param('~demo_trajectory', True)
        self.freq = freq

    def control_step(self, t, trajectory_function, speed, odom_publishers, tf_broadcaster):
        # Note: for some reason loop by value over a zip() does not work
        for i, _ in enumerate(self.strucs):
            ### Dislocate based on request
            #state_vecs[i][0] += dislocation_srv[0]
            #state_vecs[i][1] += dislocation_srv[1]
            #dislocation_srv = (0., 0.)

            # Publish odometry
            publish_structure_odometry(self.strucs[i], self.state_vecs[i], 
                                        odom_publishers, tf_broadcaster)

            if self.demo_trajectory:
                desired_state = trajectory_function(t, speed, self.trajs[i])
                # Overwrite the control input with the demo trajectory
                [thrust_newtons, roll, pitch, yaw] = position_controller(
                            self.state_vecs[i], desired_state)

            # Control output based on crazyflie input
            F_single, M_single = attitude_controller((thrust_newtons, roll, pitch, yaw), 
                                                     self.state_vecs[i])

            # Control of Moments and thrust
            F_structure, M_structure, rotor_forces = modquad_torque_control(
                            F_single, M_single, self.strucs[i], motor_sat=True)

            # Simulate
            self.state_vecs[i] = simulation_step(self.strucs[i], self.state_vecs[i], 
                            F_structure, M_structure, 1. / self.freq)
        
    def get_data_by_ind(self, index):
        if index < 0 or index > len(self.strucs):
            return None
        return self.strucs[index], self.state_vecs[index], self.trajs[index]

    def get_states(self): 
        """
        Returns list of locations of all structures 
        """
        return self.state_vecs
    
    def split_struc(self, struc_to_replace, struclist, trajlist, statelist):
        # Remove the structure that was split apart and its vars
        replace_ind = self.strucs.index(struc_to_replace)
        print("Replacing structure {} at index {}/{}".format(struc_to_replace.gen_hashstring(), replace_ind, len(self.strucs)))
        del self.strucs[replace_ind]
        del self.trajs[replace_ind]
        del self.state_vecs[replace_ind]

        # Add the new structures and assoc. vars to class instance vars
        self.strucs = self.strucs + struclist
        self.trajs = self.trajs + trajlist
        self.state_vecs = self.state_vecs + statelist
        #print(self.strucs)
        #print(self.trajs)
        #print(self.state_vecs)

        #print("set of hashstrings after split:")
        #for s in self.strucs:
        #    mat = convert_struc_to_mat(s.ids, s.xx, s.yy)
        #    print("\t{}\t{}\t{}\t{}\n{}".format(s.gen_hashstring(), s.ids, s.xx, s.yy, mat))
        #print('---')

    def disassemble(self, reconf_map):
        try:
            # Split the structure physically
            split = rospy.ServiceProxy("SplitStructure", SplitStructure)
            inp = split_srv_input_format(structure, split_dim, breakline, split_ind)
            ret = split(inp[0], inp[1], inp[2], inp[3], inp[4], inp[5], inp[6], inp[7])

            # Generate the new structures post-split (i.e. generate the actual objects)
            strucs = gen_strucs_from_split(ret)

            # Generate trajectories for new structures
            traj_vars1 = trajectory_function(0, speed, None, waypt_gen.line([state_vecs[0][0]   , state_vecs[0][1]   , state_vecs[0][2]   ], 
                                                                             [state_vecs[0][0]-15, state_vecs[0][1]-15, state_vecs[0][2]+15] ))
            traj_vars2 = trajectory_function(0, speed, None, waypt_gen.spiral(5,5,5,2, start_pt=[state_vecs[0][0], state_vecs[0][1], state_vecs[0][2]]))
            trajs = [traj_vars1, traj_vars2]
            state_vecs = [state_vecs[0], state_vecs[0]]

            # Update the structure manager
            struc_mgr.split_struc(index, strucs, trajs, state_vecs)

            # Set sim params
            t = 0
            undocked = True
        except rospy.ServiceException, e:
            print("SplitStructure service call failed: {}".format(e))
            import sys
            sys.exit(0)

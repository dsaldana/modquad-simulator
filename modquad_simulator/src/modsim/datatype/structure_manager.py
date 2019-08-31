#!/usr/bin/env python
import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from numpy import copy
import networkx as nx
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import copy

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

#thrust_newtons, roll, pitch, yaw = 0.0, 0.0, 0.0, 0.0

#def publish_for_attached_mods(robot_id, structure_x, structure_y, xx, yy, main_id, odom_publishers, tf_broadcaster):
#    publish_odom_relative(structure_x - xx[0], structure_y - yy[0], robot_id, main_id, odom_publishers[robot_id])
#    publish_transform_stamped_relative(robot_id, main_id, structure_x - xx[0], structure_y - yy[0], tf_broadcaster)
#
#def publish_structure_odometry(structure, x, odom_publishers, tf_broadcaster):
#    #global thrust_newtons, roll, pitch, yaw
#    ids, xx, yy = structure.ids, structure.xx, structure.yy
#
#    # publish main robot
#    if len(xx) == 1:
#        main_id = ids
#    else:
#        main_id = ids[0]
#    publish_odom(x, odom_publishers[main_id])
#    publish_transform_stamped(main_id, x, tf_broadcaster)
#
#    # show the other robots
#    [publish_for_attached_mods(robot_id, structure_x, structure_y, xx, yy, 
#        main_id, odom_publishers, tf_broadcaster) 
#        for robot_id, structure_x, structure_y in zip(ids, xx, yy)[1:]]

class StructureManager:
    def __init__(self, struclist):
        self.strucs = struclist # init based on those passed in
        self.demo_trajectory = rospy.get_param('~demo_trajectory', True)
        self.freq = rospy.get_param("freq", 100)
        self.state_vecs_log = [[] for _ in range(len(self.strucs))]
        self.desired_states_log = [[] for _ in range(len(self.strucs))]

    #def control_step(self, t, trajectory_function, speed, odom_publishers, tf_broadcaster):
    #    #global thrust_newtons, roll, pitch, yaw

    #    print('-----')
    #    # NOTE: for some reason loop by value over a zip() does not work
    #    for i, _ in enumerate(self.strucs):
    #        #if i < 1: continue

    #        # Publish odometry
    #        publish_structure_odometry(self.strucs[i], self.state_vecs[i], \
    #            odom_publishers, tf_broadcaster)

    #        if self.demo_trajectory:
    #            desired_state = trajectory_function(t, speed, self.trajs[i])
    #            print("pos[{}] = {}".format(i, desired_state[0]))
    #            # Overwrite the control input with the demo trajectory
    #            [thrust_newtons, roll, pitch, yaw] = position_controller(
    #                        self.state_vecs[i], desired_state)
    #            self.strucs[i].update_control_params(thrust_newtons, roll, pitch, yaw)
    #            self.desired_states_log[i].append(desired_state[0])


    #        # Control output based on crazyflie input
    #        F_single, M_single = attitude_controller(
    #                (self.strucs[i].thrust_newtons, self.strucs[i].roll, 
    #                    self.strucs[i].pitch, self.strucs[i].yaw), 
    #                self.state_vecs[i])

    #        # Control of Moments and thrust
    #        F_structure, M_structure, rotor_forces = modquad_torque_control(
    #                        F_single, M_single, self.strucs[i], motor_sat=True)

    #        # Simulate
    #        self.state_vecs_log[i].append(self.state_vecs[i][:3])
    #        self.state_vecs[i] = simulation_step(self.strucs[i], self.state_vecs[i], 
    #                        F_structure, M_structure, 1. / self.freq)
        
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
        org_des_stae = self.desired_states_log[replace_ind]
        org_stt_vecs = self.state_vecs_log[replace_ind]
        del self.desired_states_log[replace_ind]
        del self.state_vecs_log[replace_ind]

        # Add the new structures and assoc. vars to class instance vars
        self.strucs = self.strucs + struclist
        self.trajs = self.trajs + trajlist
        self.state_vecs = self.state_vecs + statelist

        self.desired_states_log += [copy.copy(org_des_stae), copy.copy(org_des_stae)]
        self.state_vecs_log += [copy.copy(org_stt_vecs), copy.copy(org_stt_vecs)]

    # Control input callback
    # Called from the main script file
    # Unsure if this is needed yet, part of trying to get disassembled structures to work properly
    #def control_input_listener(self, twist_msg):
    #    #global thrust_newtons, roll, pitch, yaw
    #    # For more info, check:
    #    # https://github.com/whoenig/crazyflie_ros
    #    self.roll = twist_msg.linear.y
    #    self.pitch = twist_msg.linear.x
    #    self.yaw = twist_msg.angular.z
    #    thrust_pwm = twist_msg.linear.z
    #
    #    c1, c2, c3 = -0.6709, 0.1932, 13.0652
    #    F_g = ((thrust_pwm / 60000. - c1) / c2) ** 2 - c3  # Force in grams
    #    if F_g<0:
    #        F_g = 0
    #
    #    self.thrust_newtons = 9.81 * F_g / 1000.  # Force in Newtons

    def make_plots(self):
        plt.style.use('dark_background')
        tstep = 1.0 / self.freq
        fig = plt.figure()
        plt.subplot(len(self.strucs), 1, 1)
        plt.title('X pos')
        for i,_ in enumerate(self.strucs):
            sveclog = np.array(self.state_vecs_log[i])
            dstatelog = np.array(self.desired_states_log[i])
            plt.subplot(len(self.strucs), 1, i+1)
            plt.plot(dstatelog[:, 0], 'c')
            plt.plot(sveclog[:, 0], 'r')
        plt.xlabel('Time (sec)')
        fig2 = plt.figure()
        plt.subplot(len(self.strucs), 1, 1)
        plt.title('Y pos')
        for i,_ in enumerate(self.strucs):
            sveclog = np.array(self.state_vecs_log[i])
            dstatelog = np.array(self.desired_states_log[i])
            plt.subplot(len(self.strucs), 1, i+1)
            plt.plot(dstatelog[:, 1], 'c')
            plt.plot(sveclog[:, 1], 'r')
        plt.xlabel('Time (sec)')
        fig3 = plt.figure()
        plt.subplot(len(self.strucs), 1, 1)
        plt.title('Z pos')
        for i,_ in enumerate(self.strucs):
            sveclog = np.array(self.state_vecs_log[i])
            dstatelog = np.array(self.desired_states_log[i])
            plt.subplot(len(self.strucs), 1, i+1)
            plt.plot(dstatelog[:, 2], 'c')
            plt.plot(sveclog[:, 2], 'r')
        plt.xlabel('Time (sec)')
        plt.show()

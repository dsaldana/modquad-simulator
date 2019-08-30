#!/usr/bin/env python
"""
This module takes a map of structures to disassemblies and executes the 
disassembly of a structure
"""
import rospy
import tf2_ros
import numpy as np
import networkx as nc
import copy

# Modquad modules
from modsim import params
from modsim.datatype.structure import Structure
from modsim.datatype.structure_manager import StructureManager
from modsim.util.undocking import gen_strucs_from_split, split_srv_input_format
from modquad_simulator.srv import SplitStructure, SplitStructureResponse

from modsim.trajectory import min_snap_trajectory

# Scheduler modules
from scheduler.gsolver import gsolve

# Interface modules
from modquad_sched_interface.interface import convert_struc_to_mat
import modquad_sched_interface.waypt_gen as waypt_gen

class DisassemblyManager:
    """
    Given structure, disassembly map, and current time t
    Plan: At what time which disassemblies happen
          Waypoints for structures to go to to avoid collision
    """
    def __init__(self, structure, reconf_map, start_time, struc_mgr, traj_func):
        self.start_time = start_time
        self.reconf_map = reconf_map
        rospy.set_param("opmode", "disassemble")
        self.mat = convert_struc_to_mat(structure.ids, structure.xx, structure.yy)
        self.next_disassemblies = {}
        self.reserve_time = 5.0
        self.time_for_disassembly = start_time + self.reserve_time # 1 seconds per layer
        self.trajectory_function = traj_func

    def plan_next_disassembly_set(self, struc_mgr):
        """
        Split all structures we currently have if they need to be split as per reconfigurability
        Reconfibgurability means can all modules be placed into the new desired location
        Note we currently do not fully do this, we simply follow the map
        """
        print("===================================================")
        speed = rospy.get_param('structure_speed', 1.0)
        self.next_disassemblies = {}

        for s in struc_mgr.strucs:
            hashstring = s.gen_hashstring()
            if hashstring in self.reconf_map:
                self.next_disassemblies[s.gen_hashstring()] = self.reconf_map[s.gen_hashstring()]

        if len(self.next_disassemblies) == 0:
            rospy.set_param("opmode", "normal")
            trajs = []
            for i,s in enumerate(struc_mgr.strucs):
                trajs.append(self.trajectory_function(0, speed, None, waypt_gen.line(struc_mgr.state_vecs[i][:3], [0.0,0.0,5.0])))
                struc_mgr.trajs = trajs
            print("Disassembly complete, returning to normal op mode")
            return

        # Get positions of all current structures
        svecs = struc_mgr.get_states()

        # Plan locations for all structures + the two new ones
        new_locs = [ [ [0.0,0.0,0.0], [0.0,0.0,0.0] ] for _ in range(len(svecs))]
        for struc,new_loc,state_vec in zip(struc_mgr.strucs, new_locs, svecs):
            dis = struc.gen_hashstring()
            if dis not in self.next_disassemblies:
                continue
            cur_loc = state_vec[:3]
            dis_loc = self.next_disassemblies[dis]
            cur_locp= [[], []]

            # Based on direction of disassembly, generate the new trajectories where 
            # generated substructures will be positioned
            if dis_loc[0][0] == 'x':
                new_loc[0] = [cur_loc[0]-1.5, cur_loc[1], cur_loc[2]]
                new_loc[1] = [cur_loc[0]+1.5, cur_loc[1], cur_loc[2]]
                cur_locp[0]= [cur_loc[0], cur_loc[1]-params.cage_width * dis_loc[0][1], cur_loc[2]]
                cur_locp[1]= [cur_loc[0], cur_loc[1]+params.cage_width * dis_loc[0][1], cur_loc[2]]
            else: # y disassembly
                new_loc[0] = [cur_loc[0], cur_loc[1]-1.5, cur_loc[2]]
                new_loc[1] = [cur_loc[0], cur_loc[1]+1.5, cur_loc[2]]
                cur_locp[0]= [cur_loc[0]-params.cage_width * dis_loc[0][1], cur_loc[1], cur_loc[2]]
                cur_locp[1]= [cur_loc[0]+params.cage_width * dis_loc[0][1], cur_loc[1], cur_loc[2]]
            print("New goals: {}".format(new_loc))

            # Split the structure physically
            split = rospy.ServiceProxy("SplitStructure", SplitStructure)
            split_dim, breakline, split_ind = dis_loc[0][0], dis_loc[0][1], dis_loc[0][2], 
            inp = split_srv_input_format(struc, int(split_dim=='y'), breakline, split_ind)
            ret = split(inp[0], inp[1], inp[2], inp[3], inp[4], inp[5], inp[6], inp[7])

            # Generate the new structures post-split (i.e. generate the actual objects)
            new_strucs = gen_strucs_from_split(ret)

            # Generate trajectories for new structures
            traj_vars1 = self.trajectory_function(0, speed, None, waypt_gen.line(cur_locp[0], new_loc[0]))
            traj_vars2 = self.trajectory_function(0, speed, None, waypt_gen.line(cur_locp[1], new_loc[1]))
            trajs = [traj_vars1, traj_vars2]

            # Generate state_vecs offset from orig. center of mass based on the split
            state_vec1 = copy.deepcopy(state_vec)
            state_vec2 = copy.deepcopy(state_vec)
            state_vec1[:3] = cur_locp[0]
            state_vec2[:3] = cur_locp[1]
            state_vecs = [state_vec1, state_vec2]

            # Update the structure manager
            struc_mgr.split_struc(struc, new_strucs, trajs, state_vecs)

            # Print new strucs
            print("Strucs as they now stand:")
            for s in struc_mgr.strucs:
                print("\t{}\t{}".format(s.ids, s.gen_hashstring()))
            print('##############')

    def take_step(self, struc_mgr, t):
        """
        Handles when to perform the next split
        """
        if t >= self.time_for_disassembly:
            self.plan_next_disassembly_set(struc_mgr)
            self.time_for_disassembly = self.reserve_time
            print("Splits at t = {}".format(t))
            return True
        return False

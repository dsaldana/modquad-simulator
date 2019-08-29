#!/usr/bin/env python
"""
This module takes a map of structures to disassemblies and executes the 
disassembly of a structure
"""
import rospy
import tf2_ros
import numpy as np
import networkx as nc

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
        self.time_for_disassembly = start_time + 5 # 5 seconds per layer
        self.trajectory_function = traj_func

        self.plan_next_disassembly_set(struc_mgr)

    def plan_next_disassembly_set(self, struc_mgr):
        print("===================================================")
        speed = rospy.get_param('structure_speed', 1.0)
        self.next_disassemblies = {}

        #print("____ BEG RECONF MAP ____")
        #for entry in self.reconf_map:
        #    print(entry)
        #print("____ END RECONF MAP ____")
        for s in struc_mgr.strucs:
            hashstring = s.gen_hashstring()
            #print(hashstring)
            if hashstring in self.reconf_map:
                self.next_disassemblies[s.gen_hashstring()] = self.reconf_map[s.gen_hashstring()]
            else:
                self.next_disassemblies[s.gen_hashstring()] = None
        print("Next disassemblies:")
        for d in self.next_disassemblies:
            print("\t{} disassemble at {}".format(d, self.next_disassemblies[d]))

        # Get positions of all current structures
        svecs = struc_mgr.get_states()
        #print("Cur Struc states: {}".format(svecs))

        # Plan locations for all structures + the two new ones
        new_locs = [ [ [0.0,0.0,0.0], [0.0,0.0,0.0] ] for _ in range(len(svecs))]
        for struc,dis,new_loc,state_vec in zip(struc_mgr.strucs, self.next_disassemblies, new_locs, svecs):
            cur_loc = state_vec[:3]
            dis_loc = self.next_disassemblies[dis]
            #print(dis)
            #print(dis_loc, new_loc, cur_loc)
            if dis_loc[0][0] == 'x':
                new_loc[0] = [cur_loc[0], cur_loc[1]-0.25, cur_loc[2]]
                new_loc[1] = [cur_loc[0], cur_loc[1]+0.25, cur_loc[2]]
            else: # y disassembly
                new_loc[0] = [cur_loc[0]-0.25, cur_loc[1], cur_loc[2]]
                new_loc[1] = [cur_loc[0]+0.25, cur_loc[1], cur_loc[2]]
            #print("After split locs: {}".format(new_locs))

            # Split the structure physically
            split = rospy.ServiceProxy("SplitStructure", SplitStructure)
            split_dim, breakline, split_ind = dis_loc[0][0], dis_loc[0][1], dis_loc[0][2], 
            inp = split_srv_input_format(struc, int(split_dim=='y'), breakline, split_ind)
            for i,val in enumerate(inp):
                print(i, val)
            ret = split(inp[0], inp[1], inp[2], inp[3], inp[4], inp[5], inp[6], inp[7])

            # Generate the new structures post-split (i.e. generate the actual objects)
            new_strucs = gen_strucs_from_split(ret)
            #for s in new_strucs:
            #    print("Printing a new struc")
            #    print(s.ids)
            #    print(s.xx)
            #    print(s.yy)
            #    print(s.motor_failure)

            # Generate trajectories for new structures
            traj_vars1 = self.trajectory_function(0, speed, None, waypt_gen.line(cur_loc, new_loc[0]))
            traj_vars2 = self.trajectory_function(0, speed, None, waypt_gen.line(cur_loc, new_loc[1]))
            trajs = [traj_vars1, traj_vars2]
            state_vecs = [state_vec, state_vec]

            # Update the structure manager
            struc_mgr.split_struc(struc, new_strucs, trajs, state_vecs)

    def take_step(self, struc_mgr, t):
        if t > self.time_for_disassembly:
            self.plan_next_disassembly_set(struc_mgr)

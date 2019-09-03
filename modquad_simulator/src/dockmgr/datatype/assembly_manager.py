#!/usr/bin/env python
"""
This module takes a set of structures and a desired final structure 
and executes the assembly
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
from modsim.util.state import init_state
from modsim.trajectory import min_snap_trajectory

# Interface modules
from modquad_sched_interface.interface import convert_struc_to_mat
import modquad_sched_interface.waypt_gen as waypt_gen

class AssemblyManager:
    """
    Given structure, disassembly map, and current time t
    Plan: At what time which disassemblies happen
          Waypoints for structures to go to to avoid collision
    """
    def __init__(self, start_time, final_struc_mat, traj_func):
        self.start_time = start_time
        rospy.set_param("opmode", "assemble")
        self.mat = final_struc_mat
        self.reserve_time = 3.0
        self.time_for_assembly = start_time + self.reserve_time # seconds per layer
        self.trajectory_function = traj_func

    def find_assembly_tree(struc_mgr):
        pass

    def plan_next_assembly_set(self, struc_mgr, t):
        """
        Split all structures we currently have if they need to be split as per reconfigurability
        Reconfibgurability means can all modules be placed into the new desired location
        Note we currently do not fully do this, we simply follow the map
        """
        #print("===================================================")
        speed = rospy.get_param('structure_speed', 1.0)
        self.next_assemblies = {}

        if len(struc_mgr.strucs) == 1:
            print("Assembly complete, returning to normal op mode")
            rospy.set_param("opmode", "normal")
            return ""

        # Plan locations for all structures + the two new ones
        new_locs = [ [ [0.0,0.0,0.0], [0.0,0.0,0.0] ] for _ in range(len(struc_mgr.strucs))]
        for struc, new_loc in zip(struc_mgr.strucs, new_locs):
            # Generate trajectories for pair of structures that will dock
            new_strucs[0].traj_vars = self.trajectory_function(
                    t, speed, None, 
                    waypt_gen.line(np.copy(cur_locp[0]), np.copy(new_loc[0])))
            new_strucs[1].traj_vars = self.trajectory_function(
                    t, speed, None, 
                    waypt_gen.line(np.copy(cur_locp[1]), np.copy(new_loc[1])))


            # Set paths of the two substructures in the struc_mgr

            # Print new strucs
            print("Strucs as they now stand:")
            for s in struc_mgr.strucs:
                print("\t{}\t{}".format(s.ids, s.gen_hashstring()))
            print('##############')
        return None

    def take_step(self, struc_mgr, t, ind):
        """
        Handles when to perform the next assembly
        """
        if t >= self.time_for_disassembly:
            ret = self.plan_next_assembly_set(struc_mgr, t)
            self.time_for_assembly += self.reserve_time
            print("Joins at t = {}, ind = {}".format(t, ind))
            return ret is None
        return False

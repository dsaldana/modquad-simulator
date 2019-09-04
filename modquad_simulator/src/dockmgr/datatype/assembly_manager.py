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
from itertools import combinations

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
    Given structure to assemble, current structures, and current time t
    Plan: At what time which assemblies happen
          Waypoints for structures to go to to avoid collision
    """
    def __init__(self, start_time, final_struc_mat, traj_func):
        self.start_time = start_time
        rospy.set_param("opmode", "assemble")
        self.mat = final_struc_mat
        self.reserve_time = 3.0
        self.time_for_assembly = start_time + self.reserve_time # seconds per layer
        self.trajectory_function = traj_func
        self.dockings = None
        self.n = rospy.get_param("num_used_robots", 2)
        self.next_plan_z = True

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
        if t >= self.next_time_to_plan:
            ret = self.plan_next_assembly_set(struc_mgr, t)
            self.time_for_assembly += self.reserve_time
            print("Joins at t = {}, ind = {}".format(t, ind))
            return ret is None
        return False

    def generate_assembly_order(self, struc_mgr):
        pass

    #def plan_next_trajs(self, t, struc_mgr, modid1, modid2, adj_dir, zlayer, traj_func):
    #    pl

    def plan_next_xy_motion(self, t, struc_mgr, modid1, modid2, adj_dir, traj_func):
        speed = rospy.get_param("structure_speed", 1.0)
        speed = 0.5
        struc1 = struc_mgr.find_struc_of_mod(modid1)
        struc2 = struc_mgr.find_struc_of_mod(modid2)

        # Start new trajectory where the structures currently are
        #waypts1 = [np.copy(struc1.state_vector[:3])]
        waypts2 = [np.copy(struc2.state_vector[:3])]
        zpos = struc2.state_vector[2]

        # We need to check that the structures are oriented relative 
        #  to each other correctly, and if not move them
        i1  = struc1.ids.index('modquad{:02d}'.format(modid1))
        i2  = struc2.ids.index('modquad{:02d}'.format(modid2))
        x1  = struc1.xx[i1]
        y1  = struc1.yy[i1]
        x2  = struc2.xx[i2]
        y2  = struc2.yy[i2]
        s1x = struc1.state_vector[0]
        s1y = struc1.state_vector[1]
        s2x = struc2.state_vector[0]
        s2y = struc2.state_vector[1]

        # World frame pos of modid1
        x1w = x1 + s1x
        y1w = y1 + s1y

        # Where struc2 should end up relative to struc1
        desire_x = 0.0 
        desire_y = 0.0

        # Whether strucs are facing right way for linear trajs to connect them
        oriented = False

        print("Trying to connect {}-{} in dir {}".format(modid1, modid2, adj_dir))

        # Compute whether oriented and what desired x,y for struc2 are
        if adj_dir == 'up':
            oriented = y1 < y2 and abs(x2 - x1) > 0.1
            desire_x = x1w
            desire_y = y1w + params.cage_width
        elif adj_dir == 'down':
            oriented = y1 > y2 and abs(x2 - x1) > 0.1
            desire_x = x1w
            desire_y = y1w - params.cage_width
        elif adj_dir == 'left':
            oriented = x1 > x2 and abs(y2 - y1) > 0.1
            desire_x = x1w - params.cage_width
            desire_y = y1w
        else: #adj_dir == 'right'
            oriented = x1 < x2 and abs(y2 - y1) > 0.1
            desire_x = x1w - params.cage_width
            desire_y = y1w

        # Get desired pos of center of mass of struc 2 
        desire_x -= x2
        desire_y -= y2
        desire_pos = np.array([desire_x, desire_y, zpos])

        # Current state
        curstate = struc2.state_vector[:3]

        print('oriented: {}'.format(oriented))
        if not oriented:
            berth = 1.0
            if adj_dir == 'up':
                waypts2.append([curstate[0] - berth, curstate[1], zpos])
                waypts2.append([curstate[0] - berth, desire_y + berth, zpos])
                waypts2.append([desire_x, desire_y + berth, zpos])
            elif adj_dir == 'down':
                waypts2.append([curstate[0] + berth, curstate[1], zpos])
                waypts2.append([curstate[0] + berth, desire_y - berth, zpos])
                waypts2.append([desire_x, desire_y - berth, zpos])
            elif adj_dir == 'left':
                waypts2.append([curstate[0], curstate[1] + berth, zpos])
                waypts2.append([desire_x - berth, curstate[1] + berth, zpos])
                waypts2.append([desire_x - berth, desire_y, zpos])
            elif adj_dir == 'right':
                waypts2.append([curstate[0], curstate[1] - berth, zpos])
                waypts2.append([desire_x + berth, curstate[1] - berth, zpos])
                waypts2.append([desire_x + berth, desire_y, zpos])

        # Finally, we want them to come together and attach
        # struc1 stays stationary, struc 2 moves in
        waypts2.append(desire_pos)

        #print("struc1 pos: {}".format(struc1.state_vector[:3]))
        #print('-------------')
        #print("struc2 pos: {}".format(struc2.state_vector[:3]))
        #print('-------------')
        #print("struc2 desire pos: {}".format(desire_pos))
        #print('-------------')
        #print("2nd set of waypts: \n{}".format(np.array(waypts2)))

        #struc1.traj_vars = traj_func(t, speed, None, np.array(waypts1))
        struc2.traj_vars = traj_func(t, speed, None, np.array(waypts2))
        #print(struc2.traj_vars.times)
        #print(struc2.traj_vars.waypts)

        # Next time we are ready, plan the z motion
        self.next_plan_z = True

    def plan_next_z_motion(self, t, struc_mgr, modid1, modid2, zlayer, traj_func):
        speed = rospy.get_param("structure_speed", 1.0)
        speed = 0.5
        struc1 = struc_mgr.find_struc_of_mod(modid1)
        struc2 = struc_mgr.find_struc_of_mod(modid2)

        # Start new trajectory where the structures currently are
        waypts1 = [np.copy(struc1.state_vector[:3])]
        waypts2 = [np.copy(struc2.state_vector[:3])]

        # Move them to be at the same z height
        nextpt1 = np.copy(struc1.state_vector[:3])
        nextpt1 += [0.1, 0.1, zlayer - struc1.state_vector[2]]
        waypts1.append(nextpt1)
        nextpt2 = np.copy(struc2.state_vector[:3])
        nextpt2 += [0.1, 0.1, zlayer - struc2.state_vector[2]]
        waypts2.append(nextpt2)

        struc1.traj_vars = traj_func(t, speed, None, np.array(waypts1))
        struc2.traj_vars = traj_func(t, speed, None, np.array(waypts2))

        self.next_time_to_plan = max([struc1.traj_vars.times[-1], struc2.traj_vars.times[-1]])
        # At next time to plan, plan xy motion
        self.next_plan_z = False

    def handle_dockings_msg(self, struc_mgr, msg, traj_func, t):
        if self.dockings is None:
            self.dockings = np.array(msg.data)
            return
        dockings = np.array(msg.data) - self.dockings
        if np.sum(dockings) == 0:
            return
        else:
            pairs = list(combinations(range(1,self.n+1), 2))
            dock_ind = np.nonzero(dockings)
            #print('-----')
            #print(msg)
            #print(dock_ind)
            for x in dock_ind[0]:
                # Find containing structures of these modules
                #print("new docking of mods: {}".format(pairs[x]))
                struc1 = struc_mgr.find_struc_of_mod(pairs[x][0])
                struc2 = struc_mgr.find_struc_of_mod(pairs[x][1])
                #print("dock struc with ids: {}".format(struc1.ids))
                #print("to dock with ids: {}".format(struc2.ids))
                if struc1.ids == struc2.ids:
                    #print("\tAlready docked")
                    continue # Already joined the relevant structures
                struc_mgr.join_strucs(struc1, struc2, pairs[x], msg.data[x], traj_func, t)
            self.dockings = np.array(msg.data) # Contains both new and old dockings
            #print('-----')

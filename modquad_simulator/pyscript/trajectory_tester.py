#!/usr/bin/env python
from time import sleep
import copy

from modsim.attitude import attitude_controller
from modsim.controller import position_controller, modquad_torque_control
from modsim.trajectory import circular_trajectory, min_snap_trajectory

from modsim.util.state import init_state

import numpy as np

from modsim.simulation.ode_integrator import simulation_step

from modsim import params
from modsim.datatype.structure import Structure
import modquad_sched_interface.waypt_gen as waypt_gen 
import modquad_sched_interface.structure_gen as structure_gen

def test_shape_with_waypts(mset, wayptset, speed=1):
    from modquad_sched_interface.interface import convert_modset_to_struc
    from scheduler.scheduler.modset import modset
    from scheduler.gsolver import gsolve
    trajectory_function = min_snap_trajectory
    traj_vars = trajectory_function(0, speed, None, wayptset)
    
    for t in np.arange(0, traj_vars.times[-1], 0.1):
        print("At t = {:04f}, desired_state = {}".format(t, trajectory_function(t, speed, traj_vars)))
        print('---')

if __name__ == '__main__':
    print("Starting simulation")
    test_shape_with_waypts(structure_gen.zero(2, 2), 
                           waypt_gen.line([0,0,0], [0,15.0,0]), 
                           #waypt_gen.rising_rect(10,10,5),
                           speed=3.5)
    print("End simulation")

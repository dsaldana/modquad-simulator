#!/usr/bin/env python
from time import sleep
import numpy as np
import matplotlib.pyplot as plt
import itertools
import sys
from gurobipy import *

from modsim.controller import position_controller
from modsim.trajectory import circular_trajectory, min_snap_trajectory
from modsim import params # cage_width

#from modsim.simulation.motion import position_controller, modquad_torque_control
#from modsim.simulation.grb_motion import grb_modquad_torque_control, grb_control_output
from modsim.util.state import init_state

from modsim.simulation.ode_integrator import simulation_step

from modsim import params
from modsim.datatype.structure import Structure

def gen_obj_func(f, mset, xx, yy, trajectory_function=min_snap_trajectory, waypts=None, pos_inds=[], mod_inds=[], rot_inds=[], t_step=0.005, tmax=10, loc=[0., 0., 0.]):
    """
    :param pi: is the gurobi vars denoting whether a module is assigned to a module position
    :param structure: is (redundant?)
    :param trajectory_function: is a function handle for chosen trajectory
    :param t_step: size of time step in seconds
    :param tmax: target time to complete trajectory
    :param loc: initial location at which structure begins
    """

    traj_vars = None
    if waypts is not None:
        traj_vars = trajectory_function(0, tmax, None, waypts)
    
    # Generate constant expressions of integrals of abs(snap)
    snapx = 0.0
    snapy = 0.0
    for t in np.arange(0, tmax, t_step):
        snap = trajectory_function(t % 10, tmax, traj_vars, ret_snap=True)
        snapx += abs(snap[0])
        snapy += abs(snap[1])
        # We do not use snapz


    # The Structure object stores module positions w.r.t center of mass
    # Generate expressions of moments about x and y axes
    Mx = LinExpr(0.0)
    My = LinExpr(0.0)
    ind = 0
    avgx = np.average(xx)
    avgy = np.average(yy)
    inds = [p for p in itertools.product(pos_inds,mod_inds,rot_inds)]
    Mx = quicksum((params.cage_width * int(i[0]) - avgx + 0.5 * pow(-1, int(k/2)) * params.chassis_width) * f[i[0], i[1], j, k] for i,j,k in inds)
    My = quicksum((params.cage_width * int(i[1]) - avgy + 0.5 * int(bool(k % 3))  * params.chassis_width) * f[i[0], i[1], j, k] for i,j,k in inds)
    print("Mx = {}".format(Mx))
    print("My = {}".format(My))

    # Combine to form the objective function
    snapsum = snapx + snapy
    expr = (snapx / snapsum) * Mx + (snapy / snapsum) * My
    #print("generated obj func: {}".format(expr))
    return expr

if __name__ == '__main__':
    print('')

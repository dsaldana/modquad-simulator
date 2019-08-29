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

def gen_obj_func(pi, mset, xx, yy, trajectory_function=min_snap_trajectory, waypts=None, pi_inds=[], rot_inds=[], t_step=0.01, speed=1, loc=[0., 0., 0.]):
    """
    :param pi: is the gurobi vars denoting whether a module is assigned to a module position
    :param structure: is (redundant?)
    :param trajectory_function: is a function handle for chosen trajectory
    :param t_step: size of time step in seconds
    :param tmax: target time to complete trajectory
    :param loc: initial location at which structure begins
    """

    traj_vars = None
    tmax = 10
    if waypts is not None:
        traj_vars = trajectory_function(0, speed, None, waypts)
        tmax = speed / traj_vars.total_dist

    
    # Generate constant expressions of integrals of abs(snap)
    snapx = 0.0
    snapy = 0.0
    for t in np.arange(0, tmax, t_step):
        snap = trajectory_function(t, speed, traj_vars, ret_snap=True)
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
    IJ = [p for p in pi_inds]
    xneg = [pow(-1, int(k/2)) for k in rot_inds]
    yneg = [pow(-1, int((k+1)%3==1)) for k in range(4)]
    Mx = (params.maxF * 
            quicksum(pi[i,j] * 
                quicksum(mset.mods[j].gamma[k] * 
                        (abs(xx[i])+0.5*params.chassis_width*xneg[k]*
                            1#(1+pow(10, xneg[k]-3))
                        ) 
                for k in rot_inds) 
            for i,j in IJ))
    My = (params.maxF * 
            quicksum(pi[i,j] * 
                quicksum(mset.mods[j].gamma[k] * 
                    (abs(yy[i])+0.5*params.chassis_width*yneg[k]*
                        1#(1+pow(10, yneg[k]-3))
                    ) 
                for k in rot_inds) 
            for i,j in IJ))
    #Mx = params.maxF * quicksum(pi[i,j] * quicksum(mset.mods[j].gamma[k] * (-xx[i]+0.5*params.chassis_width*xneg[k]*(1+pow(10, xneg[k]-3))) for k in rot_inds) for i,j in IJ)
    #My = params.maxF * quicksum(pi[i,j] * quicksum(mset.mods[j].gamma[k] * (yy[i]+0.5*params.chassis_width*yneg[k]*(1+pow(10, yneg[k]-3))) for k in rot_inds) for i,j in IJ)
    #print(Mx)
    #print(My)

    # Combine to form the objective function
    snapsum = snapx + snapy
    expr = (snapx / snapsum) * Mx + (snapy / snapsum) * My
    #print("snapx = {}".format(snapx))
    #print("snapy = {}".format(snapy))
    #print(np.nonzero(mset.struc)[0])
    #print(np.nonzero(mset.struc)[1])

    return expr

if __name__ == '__main__':
    print('You should not need to directly running obfunc_gen.py?')

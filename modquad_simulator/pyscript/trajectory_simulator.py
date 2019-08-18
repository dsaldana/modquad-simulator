#!/usr/bin/env python
from time import sleep, localtime, strftime
import copy

from modsim.attitude import attitude_controller
from modsim.controller import position_controller, modquad_torque_control
from modsim.trajectory import circular_trajectory, min_snap_trajectory

from modsim.util.state import init_state

import numpy as np

from modsim.simulation.ode_integrator import simulation_step

from modsim import params
from modsim.datatype.structure import Structure
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

#plt.style.use('dark_background')

fig = plt.figure()
fig2 = plt.figure()

def simulate(structure, trajectory_function, t_step=0.01, speed=1, loc=[1., .0, .0], 
        waypts=None, figind=1, filesuffix=""):
    """
    :param structure:
    :param trajectory_function:
    :param t_step: time step
    :param tmax: maximum time
    """
    state_vector = init_state(loc, 0)
    state_log = []
    forces_log = []
    pos_err_log = [0,0,0]
 
    traj_vars = None
    #print("speed0 = {}".format(speed))
    tmax = 10
    if waypts is not None:
        traj_vars = trajectory_function(0, speed, None, waypts)
        tmax = traj_vars.total_dist / speed

    xmin = np.min(traj_vars.waypts[:, 0])-3
    xmax = np.max(traj_vars.waypts[:, 0])+3
    ymin = np.min(traj_vars.waypts[:, 1])-3
    ymax = np.max(traj_vars.waypts[:, 1])+3
    zmin = np.min(traj_vars.waypts[:, 2])-3
    zmax = np.max(traj_vars.waypts[:, 2])+3

    overtime = 1.0
    lw=3
    alphaset = 0.8
    ax = fig2.add_subplot(1,1,1, projection='3d')
    ax.plot(traj_vars.waypts[:,0], traj_vars.waypts[:,1], traj_vars.waypts[:,2], zdir='z', color='b', linewidth=lw)
    #ax.plot([0, 1],[0, 1], [0, 1], zdir='z', color='k')
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_zlim(zmin, zmax)
    #plt.ion()
    #plt.show()
    # For every time step
    ind = 0
    for t in np.arange(0, overtime * tmax, t_step):
        # print t, state_vector
        ##### Trajectory
        desired_state = trajectory_function(t, speed, traj_vars)
        #print("t = {}, desired state = {}".format(t, desired_state))
        # Position controller for a single robot
        [thrust_newtons, roll, pitch, yaw] = position_controller(state_vector, desired_state)
        F, M = attitude_controller((thrust_newtons, roll, pitch, yaw), state_vector)
        #print(desired_state[0] - state_vector[:3])
        pos_err_log += np.power(desired_state[0] - state_vector[:3], 2)

        # Structure control
        F_structure, M_structure, rotor_forces = modquad_torque_control(F, M, structure, motor_sat=True)
        forces_log.append(rotor_forces)


        # Simulate
        new_state_vector = simulation_step(structure, state_vector, F_structure, M_structure, t_step)
        if new_state_vector is None:
            break
        state_vector = new_state_vector

        state_log.append(np.copy(state_vector))

        #ax.scatter(state_vector[0], state_vector[1], state_vector[2], zdir='z', color='r', linewidth=lw, marker='.')
        #plt.draw()
        #plt.pause(0.010)
        ind += 1
    pos_err_log /= ind
    pos_err_log = np.sqrt(pos_err_log)

    print("end simulation")
    integral_val = np.sum(np.array(forces_log) ** 2) * t_step
    if figind < 1:
        print("total integral={}".format(integral_val))
        return integral_val
    #ax.grid()

    state_log = np.array(state_log)
    ax.plot(state_log[:, 0], state_log[:, 1], state_log[:, 2], zdir='z', color='r', linewidth=lw)
    ax.legend(["Planned Path", "Actual Path"])
    #strftime("%Y-%m-%d_%H:%M:%S", localtime()), 
    plt.savefig("figs/3d_{}.pdf".format(filesuffix))
    plt.sca(fig.gca())

    tstate = np.arange(0, overtime * tmax, t_step)
    twaypt = np.arange(0, tmax + 0.5, 0.5)

    # Generate legend
    legend_vals = ["Actual path", "Desired path"]
    # Shrink current axis's height by 10% on the bottom
    ax2 = plt.gca()
    box = ax.get_position()
    ax2.set_position([box.x0, box.y0 + box.height * 0.1,
                         box.width, box.height * 0.9])

    ylabelsize = 12
    # Plot first one so that we can do legend
    plt.subplot(4,1,figind+0)
    plt.plot(tstate, state_log[:, 0], color='r', linewidth=lw)
    plt.plot(twaypt, traj_vars.waypts[:,0], alpha=alphaset, color='g', linewidth=lw)
    plt.ylabel("X position\n(m)", size=ylabelsize)
    plt.gca().set_ylim(xmin, xmax)
    plt.gca().xaxis.set_ticklabels([])
    plt.grid()

    # Put a legend below current axis
    plt.figlegend(legend_vals, loc='upper center', ncol=2)#bbox_to_anchor=(0.5,  0.95),
                      #fancybox=True, shadow=True, ncol=2)

    plt.subplot(4,1,figind+1)
    plt.plot(tstate, state_log[:, 1], color='r', linewidth=lw)
    plt.plot(twaypt, traj_vars.waypts[:,1], alpha=alphaset, color='g', linewidth=lw)
    plt.ylabel("Y position\n(m)", size=ylabelsize)
    plt.gca().set_ylim(ymin, ymax)
    plt.gca().xaxis.set_ticklabels([])
    plt.grid()

    plt.subplot(4,1,figind+2)
    plt.plot(tstate, state_log[:, 2], color='r', linewidth=lw)
    plt.plot(twaypt, traj_vars.waypts[:,2], alpha=alphaset, color='g', linewidth=lw)
    plt.ylabel("Z position\n(m)", size=ylabelsize)
    plt.gca().set_ylim(zmin, zmax)
    plt.gca().xaxis.set_ticklabels([])
    plt.grid()

    # sum of the squared forces
    plt.subplot(4,1,figind+3)
    plt.xlabel("Time (sec)")
    plt.ylabel("Force\n(N)", size=ylabelsize)
    forces_log = forces_log[5:]
    plt.plot(tstate[5:], np.sum(np.array(forces_log) ** 2, axis=1), color='r', linewidth=lw)
    plt.gca().set_ylim(0, 0.75)
    plt.grid()
    #strftime("%Y-%m-%d_%H:%M:%S", localtime()), 
    plt.savefig("figs/2d_{}.pdf".format(filesuffix))
    print("total integral={}".format(np.sum(np.array(forces_log) ** 2) * t_step))
    plt.clf() # Clear figures
    plt.sca(ax)
    plt.clf()
    return integral_val, pos_err_log

def circ_traj_test(): 
    structure = Structure(ids=['modquad01', 'modquad02'], xx=[0, params.cage_width], yy=[0, 0], motor_failure=[])

    # w = params.cage_width
    # structure = Structure(ids=['1', '2', '3', '4'], xx=[0., 0., -w, -w], yy=[0., -w, -w, 0.])
    # structure = Structure()
    structure4 = Structure(ids=['modquad01', 'modquad02'],
                           xx=[0, params.cage_width, 0, params.cage_width],
                           yy=[0, 0, params.cage_width, params.cage_width],
                           motor_failure=[(1, 2)])

    trajectory_function = circular_trajectory

    simulate(structure4, trajectory_function)

def test_shape_with_waypts(mset, wayptset, speed=1, test_id=""):
    from modquad_sched_interface.interface import convert_modset_to_struc
    from scheduler.scheduler.modset import modset
    from scheduler.gsolver import gsolve
    trajectory_function = min_snap_trajectory
    traj_vars = trajectory_function(0, speed, None, wayptset)

    # Run Set 1
    gsolve(mset, waypts=traj_vars.waypts)
    mset.fault_rotor(0,0)
    mset.fault_rotor(0,1)
    #mset.fault_rotor(0,2)
    mset.fault_rotor(2,3)
    struc1 = convert_modset_to_struc(mset)
    forces1, pos_err1 = simulate(struc1, trajectory_function, waypts=wayptset, loc=[0,0,0], figind=1, speed=speed, filesuffix="{}_noreform".format(test_id))

    tmp = copy.deepcopy(mset)
    #tmp.fault_rotor(2, 3)
    gsolve(tmp, waypts=traj_vars.waypts)
    struc2 = convert_modset_to_struc(tmp)
    forces2, pos_err2 = simulate(struc2, trajectory_function, waypts=wayptset, loc=[0,0,0], figind=1, speed=speed, filesuffix="{}_reform".format(test_id))
    return [forces1, forces2, pos_err1, pos_err2]

if __name__ == '__main__':
    import modquad_sched_interface.waypt_gen as waypt_gen
    import modquad_sched_interface.structure_gen as structure_gen
    print("Starting simulation")
    forces = []
    pos_err = []
    print(waypt_gen.zigzag_xy(3,3))
    for i in range(5,  6, 2):
        results = test_shape_with_waypts(
                           #structure_gen.zero(3, 3), 
                           structure_gen.square(4), 
                           #waypt_gen.line([0,0,0], [i,i,i]), 
                           waypt_gen.rect(10,10),
                           #waypt_gen.zigzag_xy(10,5),
                           #waypt_gen.spiral(i,i,i,2),
                           speed=1.25, test_id="motorsat2_rect10x10_4x4full")
        forces.append(results[0:2])
        pos_err.append(results[2:])
    for f in forces:
        print(f, f[1]-f[0])
    for p in pos_err[0]:
        print(np.average(p))
        #print("Forces = {}, change in force = {}".format(forces, forces[1]-forces[0]))

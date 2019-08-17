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
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

plt.style.use('dark_background')

fig = plt.figure()
fig2 = plt.figure()

def simulate(structure, trajectory_function, t_step=0.01, speed=1, loc=[1., .0, .0], 
        waypts=None, figind=1):
    """
    :param structure:
    :param trajectory_function:
    :param t_step: time step
    :param tmax: maximum time
    """
    state_vector = init_state(loc, 0)
    state_log = []
    forces_log = []
 
    traj_vars = None
    #print("speed0 = {}".format(speed))
    tmax = 10
    if waypts is not None:
        traj_vars = trajectory_function(0, speed, None, waypts)
        tmax = traj_vars.total_dist / speed
    overtime = 1.0
    lw=3
    alphaset = 0.8
    ax = fig2.add_subplot(1,1,1, projection='3d')
    ax.plot(traj_vars.waypts[:,0], traj_vars.waypts[:,1], traj_vars.waypts[:,2], zdir='z', color='b', linewidth=lw)
    ax.plot([0, 1],[0, 1], [0, 1], zdir='z', color='k')
    ax.legend(["Desired Path", "Actual Path"])
    ax.set_xlim(-5, 20)
    ax.set_ylim(-5, 20)
    ax.set_zlim(-5, 20)
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

        # Structure control
        F_structure, M_structure, rotor_forces = modquad_torque_control(F, M, structure, motor_sat=False)
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

    print("end simulation")
    if figind < 1:
        print("total integral={}".format(np.sum(np.array(forces_log) ** 2) * t_step))
        return
    #ax.grid()

    state_log = np.array(state_log)
    ax.plot(state_log[:, 0], state_log[:, 1], state_log[:, 2], zdir='z', color='r', linewidth=lw)
    plt.sca(fig.gca())

    tstate = np.arange(0, overtime * tmax, t_step)
    twaypt = np.arange(0, tmax + 0.5, 0.5)

    plt.subplot(4,1,figind+0)
    plt.plot(tstate, state_log[:, 0], color='r', linewidth=lw)
    plt.plot(twaypt, traj_vars.waypts[:,0], alpha=alphaset, color='g', linewidth=lw)
    plt.ylabel("X position")
    plt.legend(["Actual path", "Desired path"])
    plt.gca().set_ylim(-5, 20)
    plt.grid()

    plt.subplot(4,1,figind+1)
    plt.plot(tstate, state_log[:, 1], color='r', linewidth=lw)
    plt.plot(twaypt, traj_vars.waypts[:,1], alpha=alphaset, color='g', linewidth=lw)
    plt.ylabel("Y position")
    plt.gca().set_ylim(-5, 20)
    plt.grid()

    plt.subplot(4,1,figind+2)
    plt.plot(tstate, state_log[:, 2], color='r', linewidth=lw)
    plt.plot(twaypt, traj_vars.waypts[:,2], alpha=alphaset, color='g', linewidth=lw)
    plt.ylabel("Z position")
    plt.gca().set_ylim(-5, 20)
    plt.grid()

    # sum of the squared forces
    plt.subplot(4,1,figind+3)
    plt.xlabel("Time (sec)")
    plt.ylabel("Force (N)")
    forces_log = forces_log[5:]
    plt.plot(tstate[5:], np.sum(np.array(forces_log) ** 2, axis=1), color='r', linewidth=lw)
    plt.grid()
    print("total integral={}".format(np.sum(np.array(forces_log) ** 2) * t_step))

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

def test_shape_with_waypts(mset, wayptset, speed=1):
    from modquad_sched_interface.interface import convert_modset_to_struc
    from scheduler.scheduler.modset import modset
    from scheduler.gsolver import gsolve
    trajectory_function = min_snap_trajectory
    traj_vars = trajectory_function(0, speed, None, wayptset)

    # Run Set 1
    gsolve(mset, waypts=traj_vars.waypts)
    #mset.fault_rotor(0,0)
    struc1 = convert_modset_to_struc(mset)
    simulate(struc1, trajectory_function, waypts=wayptset, loc=[0,0,0], figind=1, speed=speed)

    #tmp = copy.deepcopy(mset)
    ##tmp.fault_rotor(2, 3)
    #gsolve(tmp, waypts=traj_vars.waypts)
    #struc2 = convert_modset_to_struc(tmp)
    #simulate(struc2, trajectory_function, waypts=wayptset, loc=[0,0,0], figind=3, speed=speed)

if __name__ == '__main__':
    import modquad_sched_interface.waypt_gen as waypt_gen
    import modquad_sched_interface.structure_gen as structure_gen
    print("Starting simulation")
    test_shape_with_waypts(structure_gen.zero(3, 3), 
                           waypt_gen.line([0,0,0], [15.0,15.0,15.0]), 
                           #waypt_gen.rising_rect(10,10,5),
                           speed=3.5)
    plt.show()

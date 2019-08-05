from scipy.integrate import ode
from modsim.simulation.motion import state_derivative

from modsim.controller import control_handle
from modsim.trajectory import circular_trajectory

from modsim.simulation.motion import control_output


def simulation_step(structure, state_vector, F, M, time_step):
    # ##### Trajectory
    # desired_state = circular_trajectory(t % 10, 10)
    # # Position controller
    # [F, M] = control_output(t, state_vector, desired_state, control_handle)

    ## Derivative of the robot dynamics
    f_dot = lambda t1, s: state_derivative(s, F, M, structure)

    # Solve the differential equation of motion
    r = ode(f_dot).set_integrator('dopri5', nsteps=5000)
    r.set_initial_value(state_vector, 0)
    r.integrate(time_step, step=True)
    if not r.successful():
        print 'Error trying to integrate'
        return None
    state_vector = r.y

    # Simulate floor. Coordinate z in position is always greater than zero.
    if state_vector[2] < 0:
        state_vector[2] = 0.
        # Velocity towards the floor
        if state_vector[5] < 0:
            state_vector[5] = 0.

    return state_vector

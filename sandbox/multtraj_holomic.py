# Summary: Discrete grid of waypoints that considers a set of initial/final conditions


from omgtools import *

def ComputeCostMatrix(WPi, WPf, nv_vel_steps):
    # Robot configuration
    [v_min, v_max] = [-.5, .5]
    [a_min, a_max] = [-1, 1]

    # Possible departure/arrival configurations
    theta = np.linspace(0, 2*np.pi, nb_vel_steps + 1 )
    vel = v_max*np.array([np.cos(theta[0:-1]), np.sin(theta[0:-1])])

    # Cost Matrix # TODO: how to organize the data structure for this?
    # right now: cost = [theta_init theta_term duration length]
    cost = np.zeros([nb_vel_steps**2, 4])

    ## OMG-tools
    # Environment
    environment = Environment(room={'shape': Square(5.)})  # TODO: this shouldn't be a fixed size..
    # Evaluate for each initial and final velocity constraint 
    k = 0
    for v_init in range(0, nb_vel_steps):
        for v_term in range(0, nb_vel_steps):
            # Creating holonomic vehicle instance
            vehicle =  Holonomic(bounds={'vmin': v_min, 'vmax': v_max, 'amin': a_min, 'amax': a_max})
            vehicle.set_initial_conditions(WPi, vel[:, v_init].reshape(2,1) )
            vehicle.set_terminal_conditions(WPf, vel[:, v_term].reshape(2,1) )
            # Construct the problem
            problem = Point2point(vehicle, environment, freeT=True)
            problem.init()
            # Set simulator
            simulator = Simulator(problem)
            problem.plot('scene')        # Comment this line if the plot annoys you
            # solve
            trajectories, signals = simulator.run()
            # Compute metrics
            T=trajectories['time'][-1][-1][-1] # for trajectory time
            state = trajectories['state'][0]   # for trajectory length..
            diff = [np.linalg.norm( np.array([state[0][i+1],state[1][i+1]]) - np.array([state[0][i],state[1][i]])) for i in range(state.shape[1] - 1)]
            traj_length = np.sum(diff)
            # Update cost matrix
            cost[k][0] = theta[v_init]
            cost[k][1] = theta[v_term]
            cost[k][2] = T
            cost[k][3] = traj_length
            k += 1

    return cost

# Waypoints
WPi = [0., 0.]
WPf = [1., 1.]
nb_vel_steps = 2
# Compute the cost
cost = ComputeCostMatrix(WPi, WPf, nb_vel_steps)
# Print and save result
print cost
np.savetxt("cost.txt", cost, fmt="%.2f")
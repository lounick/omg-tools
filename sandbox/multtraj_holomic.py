# Summary: Discrete grid of waypoints that considers a set of initial/final conditions
# for a holonomic vehicle


from omgtools import *

def ComputeCostMatrix(WPi, WPf, nv_vel_steps):
    # Robot configuration
    [v_min, v_max] = [-.5, .5]
    [a_min, a_max] = [-1, 1]

    # Possible departure/arrival configurations
    theta = np.linspace(0, 2*np.pi, nb_vel_steps + 1 )
    vel = v_max*np.array([np.cos(theta[0:-1]), np.sin(theta[0:-1])])

    # Cost Matrix # TODO: how to organize the data structure for this?
    # right now: cost = [WPi_x WPi_y WPf_x WPf_y theta_init theta_term duration length]
    cost = np.zeros([nb_vel_steps**2, 8])

    ## OMG-tools
    # Environment
    environment = Environment(room={'shape': Square(14.), 
                                    'position': [6, 6]})  # TODO: this shouldn't be a fixed size..
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
            cost[k][0:2] = WPi
            cost[k][2:4] = WPf
            cost[k][4] = v_init #theta[v_init]
            cost[k][5] = v_term #theta[v_term]
            cost[k][6] = T
            cost[k][7] = traj_length
            k += 1

    return cost

'''
# Example for 2 Waypoints
WPi = [0., 0.]
WPf = [9., 9.]
nb_vel_steps = 2
# Compute the cost
cost = ComputeCostMatrix(WPi, WPf, nb_vel_steps)
## Print and save result
print cost
np.savetxt("cost.txt", cost, fmt="%.2f")
'''

bl_wp = np.array([0,0])  # bottom left waypoint
ur_wp = np.array([9,9])  # upper right waypoint
size = [10,10] # nb of elements in the (x,y) axes
nb_vel_steps = 8
x = np.linspace(bl_wp[0], ur_wp[0], size[0])
y = np.linspace(bl_wp[1], ur_wp[1], size[1])

#tic = time.time()
for i in range(0,size[0]):
    for j in range(0,size[1]):
        WPi = [x[i], y[j]]
        for ii in range(0,size[0]):
            for jj in range(0,size[1]):
                if (i==ii and j==jj):
                    continue
                WPf = [x[ii], y[jj]]
                cost = ComputeCostMatrix(WPi, WPf, nb_vel_steps)
                if 'total_cost' in locals():
                    total_cost = np.vstack([total_cost, cost])
                else:
                    total_cost = cost

#print time.time() - tic
np.savetxt("cost.txt", total_cost, fmt="%.2f")

            


# Summary: Discrete grid of waypoints that considers a set of initial/final conditions
# for a holonomic vehicle


from omgtools import *

def ComputeCostMatrix(WP, init, final, nv_vel_steps):
    # Robot configuration
    [v_min, v_max] = [-.5, .5]
    [a_min, a_max] = [-1, 1]

    # Possible departure/arrival configurations
    theta = np.linspace(0, 2*np.pi, nb_vel_steps + 1 )
    vel = .95*v_max*np.array([np.cos(theta[0:-1]), np.sin(theta[0:-1])])

    # Cost Matrix # TODO: how to organize the data structure for this?
    # right now: cost = [WPi WPf theta_init theta_term duration length]
    cost = np.zeros([nb_vel_steps**2, 6])

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
            #vehicle =  Holonomic(options={'syslimit':'norm_2'})
            vehicle.set_initial_conditions(WP[:,init], vel[:, v_init].reshape(2,1) )
            vehicle.set_terminal_conditions(WP[:,final], vel[:, v_term].reshape(2,1) )
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
            cost[k][0] = init
            cost[k][1] = final
            cost[k][2] = v_init #theta[v_init]
            cost[k][3] = v_term #theta[v_term]
            cost[k][4] = T
            cost[k][5] = traj_length
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

cost_file=open('cost.txt','w+')
bl_wp = np.array([0,0])  # bottom left waypoint
ur_wp = np.array([2,2])  # upper right waypoint
size = [2,2] # nb of elements in the (x,y) axes
nb_vel_steps = 1
xx, yy = np.meshgrid(np.linspace(bl_wp[0], ur_wp[0], size[0]), 
                    np.linspace(bl_wp[1], ur_wp[1], size[1]) )

numel_WP = size[0]*size[1]  # number of waypoints in the grid
WP = np.array( [ xx.reshape(1,numel_WP)[0], yy.reshape(1,numel_WP)[0] ] )

tic = time.time()
for i in range(0,numel_WP):
    for f in range(0,numel_WP):
        if(i==f):
            continue       
        cost = ComputeCostMatrix(WP, i, f, nb_vel_steps)
        for k in range(0, cost.shape[0]):
            cost_file.write( ' '.join(map(str,cost[k])) + '\n' )

print "Total time: ", time.time() - tic

cost_file.close()
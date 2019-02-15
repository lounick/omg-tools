# Summary: Discrete grid of waypoints that considers a set of initial/final conditions
# for a holonomic vehicle


from omgtools import *

def ComputeCostMatrix(WP, init, final, nv_vel_steps):
    # Robot configuration
    [v_min, v_max] = [-.5, .5]
    [a_min, a_max] = [-1, 1]

    # Possible departure/arrival configurations
    theta = np.linspace(0, 2*np.pi, nb_vel_steps + 1 )
    vel_vec_i = v_max*np.array([np.cos(theta[0:-1]), np.sin(theta[0:-1])])
    vel_vec_f = v_max*np.array([np.cos(theta[0:-1]), np.sin(theta[0:-1])])
    if init <= 0:
        vel_vec_i = vel_vec_i*0
    if final >= np.size(WP)/2-1: 
        vel_vec_f = vel_vec_f*0

    # Cost Matrix # TODO: how to organize the data structure for this?
    # right now: cost = [WPi WPf theta_init theta_term duration length v_cost]
    cost = np.zeros([nb_vel_steps**2, 8])

    ## OMG-tools
    # Environment
    environment = Environment(room={'shape': Square(30.), 
                                    'position': [-5, -5]})  # TODO: this shouldn't be a fixed size..
    # Evaluate for each initial and final velocity constraint 
    k = 0
    for v_init in range(0, nb_vel_steps):
        for v_term in range(0, nb_vel_steps):
            # Creating holonomic vehicle instance
            vehicle =  Holonomic(bounds={'vmin': v_min, 'vmax': v_max, 'amin': a_min, 'amax': a_max})
            #vehicle =  Holonomic(options={'syslimit':'norm_2'})
            vehicle.set_initial_conditions(WP[:,init], vel_vec_i[:, v_init].reshape(2,1) )
            vehicle.set_terminal_conditions(WP[:,final], vel_vec_f[:, v_term].reshape(2,1) )
            # Construct the problem
            problem = Point2point(vehicle, environment, freeT=True)
            problem.set_options({'verbose': 0})
            problem.init()
            # Set simulator
            simulator = Simulator(problem)
            #problem.plot('scene')        # Comment this line if the plot annoys you
            # solve
            trajectories, signals = simulator.run()

            # Compute metrics
            T=trajectories['time'][-1][-1][-1]  # for trajectory time
            dt = trajectories['time'][0][0][1] - trajectories['time'][0][0][0]
            state = trajectories['state'][0]  # x,y position
            vel = trajectories['input'][0]  # x,y velocities  
            acc = trajectories['dinput'][0]  # x,y accelerations  
            
            diff = [np.linalg.norm( np.array([state[0][i+1],state[1][i+1]]) - np.array([state[0][i],state[1][i]])) for i in range(state.shape[1] - 1)]
            traj_length = np.sum(diff)
            v_cost = sum(sum(vel*vel, 0)*dt)
            acc_cost = sum(sum(acc*acc, 0)*dt)
            # Update cost matrix
            cost[k][0] = init
            cost[k][1] = final
            cost[k][2] = v_init #theta[v_init]
            cost[k][3] = v_term #theta[v_term]
            cost[k][4] = T
            cost[k][5] = traj_length
            cost[k][6] = v_cost   
            cost[k][7] = acc_cost               
            k += 1

    return cost

cost_file=open('cost.txt','w+')
bl_wp = np.array([0,0])  # bottom left waypoint
ur_wp = np.array([6,6])  # upper right waypoint
size = [7,7] # nb of elements in the (x,y) axes
nb_vel_steps = 4
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

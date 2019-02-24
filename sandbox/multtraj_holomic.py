# Summary: Discrete grid of waypoints that considers a set of initial/final conditions
# for a holonomic vehicle


from omgtools import *
import pickle

def ComputeCostMatrix(dist, grid_size, v_max):
    ## OMG-tools
    # Robot configuration
    vehicle = Holonomic()
    vehicle.set_options({'ideal_prediction': False})
    vehicle.set_initial_conditions([0., 0.],[0., 0.]) # dummy: required for problem.init()
    vehicle.set_terminal_conditions([0., 0.],[0., 0.]) # dummy: required for problem.init()
    # Environment
    environment = Environment(room={'shape': Square(20.), 'position': [-5, -5]}) 
    # create a point-to-point problem
    problem = Point2point(vehicle, environment, freeT=True)
    problem.set_options({'verbose': 0})
    problem.init()    
    # create deployer
    update_time = .1
    sample_time = 0.01
    current_time = 0
    current_state = [.0, .0]
    deployer = Deployer(problem, sample_time, update_time)

    # Velocity vector
    theta = np.linspace(0, 2*np.pi, 2**(grid_size+1) + 1 )
    vel = v_max*np.array([np.cos(theta[0:-1]), np.sin(theta[0:-1])])
    vel = np.concatenate( (np.zeros([2,1]), vel ),axis=1 )

    # Cost Matrix
    cost = np.zeros([grid_size,grid_size, 2**(grid_size+1)+1, 2**(grid_size+1)+1])
   
    for x in range(1, grid_size):
        for y in range(0, x+1):
            for v_init in range(0, 2**(grid_size+1)+1):
                for v_final in range(0, 2**(grid_size+1)+1):
                    #if (v_init == 0 and v_final == 0):
                    #    continue
                    state_traj = np.c_[current_state]
                    input_traj = np.c_[vel[:, v_init]]
                    wpf = np.array([np.sqrt(x*x+y*y),0.])*dist
                    # # Creating holonomic vehicle instance
                    vehicle.set_initial_conditions([0,0], vel[:,v_init])
                    vehicle.set_terminal_conditions(wpf, vel[:, v_final])
                    # # Construct the problem
                    # #vehicle.set_initial_conditions(via_point) # for init guess
                    deployer.reset() # let's start from new initial guess
                    # # update motion planning
                    trajectories = deployer.update(current_time, current_state, input_traj, None, None, False, True)
                    # # store state & input trajectories -> simulation of ideal trajectory following
                    state = np.c_[state_traj, trajectories['state'][:, :]]
                    diff = [np.linalg.norm( np.array([state[0][i+1],state[1][i+1]]) - np.array([state[0][i],state[1][i]])) for i in range(state.shape[1] - 1)]
                    traj_length = np.sum(diff)
                    T=trajectories['time'][0][-1]  # for trajectory time
                    cost[x,y,v_init,v_final] = traj_length                   
    return cost

# Grid parameters    
size = 3
dist = 1
# Vehicle parameters
v_max = .5

cost = ComputeCostMatrix(dist, size, v_max)
file = open('cost', 'wb')
pickle.dump(cost, file)
file.close()

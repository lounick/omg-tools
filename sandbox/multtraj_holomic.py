# Summary: Discrete grid of waypoints that considers a set of initial/final conditions
# for a holonomic vehicle


from omgtools import *
import pickle
import time

def ComputeCostMatrix(dist, grid_size, vel_steps, v_max):
    ## OMG-tools
    # Robot configuration
    #vehicle = Holonomic(options={'syslimit': 'norm_2'})
    vehicle = Holonomic()
    vehicle.set_options({'ideal_prediction': False})
    vehicle.set_initial_conditions([0., 0.],[0., 0.]) # dummy: required for problem.init()
    vehicle.set_terminal_conditions([0., 0.],[0., 0.]) # dummy: required for problem.init()
    # Environment
    environment = Environment(room={'shape': Square(40.), 'position': [-10, -10]}) 
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
    theta = np.linspace(0, 2*np.pi, vel_steps + 1 )
    vel = v_max*np.array([np.cos(theta[0:-1]), np.sin(theta[0:-1])])
    vel = np.concatenate( (np.zeros([2,1]), vel ),axis=1 )

    # Cost Matrix
    cost = np.zeros([grid_size,grid_size, vel_steps+1, vel_steps+1])
   
    for x in range(1, grid_size):
        for y in range(0, x+1):
            for v_init in range(0, vel_steps+1):
                for v_final in range(0, vel_steps+1):
                    #if (v_init == 0 and v_final == 0):
                    #    continue
                    state_traj = np.c_[current_state]
                    input_traj = np.c_[vel[:, v_init]]
                    wpf = np.array([x,y])*dist
                    # # Creating holonomic vehicle instance
                    vehicle.set_initial_conditions([0,0], vel[:,v_init])
                    vehicle.set_terminal_conditions(wpf, vel[:, v_final])
                    # # Construct the problem
                    # #vehicle.set_initial_conditions(via_point) # for init guess
                    deployer.reset() # let's start from new initial guess
                    # # update motion planning
                    trajectories = deployer.update(current_time, current_state, input_traj, None, None, False, True)
                    # # store state & input trajectories -> simulation of ideal trajectory following
                    t = trajectories['time'][0] 
                    T= trajectories['time'][0][-1]  # for trajectory time
                    dt = trajectories['time'][0][1] - trajectories['time'][0][0] 
                    # Position
                    pos = np.c_[state_traj, trajectories['state'][:, :]]
                    diff = [np.linalg.norm( np.array([pos[0][i+1],pos[1][i+1]]) - np.array([pos[0][i],pos[1][i]])) for i in range(pos.shape[1] - 1)]
                    cost_traj_length = np.sum(diff)
                    # Velocity
                    input_traj = np.c_[trajectories['input'][:, :]]
                    input_traj_abs = np.sqrt(sum(input_traj*input_traj))
                    cost_vel_cube = dt*sum(input_traj_abs**3)
                    # Acceleration
                    dinput_traj = np.c_[trajectories['dinput'][:, :]]
                    dinput_traj_abs = np.sqrt(sum(dinput_traj*dinput_traj))
                    cost_acc_vel = dt*sum(dinput_traj_abs*input_traj_abs)
                    #print x,y,v_init,v_final, cost_traj_length                                 
                    cost[x,y,v_init,v_final] = cost_vel_cube + cost_acc_vel
    return cost

# Grid parameters    
grid_size = 8
vel_steps = 8
dist = 1
# Vehicle parameters
v_max = .5

tic = time.time()
cost = ComputeCostMatrix(dist, grid_size, vel_steps, v_max)
toc = time.time()
file = open('cost', 'wb')
pickle.dump(cost, file)
file.close()
print toc - tic
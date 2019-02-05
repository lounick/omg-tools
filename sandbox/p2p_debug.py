# Summary: Debug file
# Error: After calling the function ComputeCostMatrix for XYZ times, the
# OMG optilayer fails.


from omgtools import *

def ComputeCostMatrix(WPi, WPf, Vi, Vf):
    ## OMG-tools
    # Environment
    # Evaluate for each initial and final velocity constraint 
    environment = Environment(room={'shape': Square(5.), 
                                    'position': [0, 0]})  # TODO: this shouldn't be a fixed size..
    # Creating holonomic vehicle instance
    vehicle =  Holonomic()
    # Setting initial values
    vehicle.set_initial_conditions(WPi, Vi )
    vehicle.set_terminal_conditions(WPi, Vf)
    # Construct the problem
    problem = Point2point(vehicle, environment, freeT=True)
    problem.set_options({'verbose': 0})
    problem.init()
    # Set simulator
    simulator = Simulator(problem)
    # solve
    trajectories, signals = simulator.run() 

N = 1000       # Number of iterations
WPi = [0,0]    # Waypoints
WPf = [1,1]
Vi = [0,0]     # Velocity assignment
Vf = [0,0.5]
for i in range(0, N):
    print "Iteration %d" % (i)
    cost = ComputeCostMatrix(WPi, WPf, Vi, Vf)
       

# Summary: Debug file
# Error: After calling the function ComputeCostMatrix for XYZ times, the
# OMG optilayer fails.


#from omgtools import *
import omgtools




def ComputeCostMatrix(WPi, WPf, Vi, Vf, environment):
    ## OMG-tools
    # Environment
    # Evaluate for each initial and final velocity constraint 
    # Creating holonomic vehicle instance
    vehicle =   omgtools.Holonomic()
    # Setting initial values
    vehicle.set_initial_conditions(WPi, Vi )
    vehicle.set_terminal_conditions(WPf, Vf)
    # Construct the problem
    problem =  omgtools.Point2point(vehicle, environment, freeT=True)
    problem.set_options({'verbose': 0})
    problem.init()
    # Set simulator
    simulator = omgtools.Simulator(problem)
    # solve
    trajectories, signals = simulator.run()



N = 1000       # Number of iterations
WPi = [0,0]    # Waypoints
WPf = [1,1]
Vi = [0,0]     # Velocity assignment
Vf = [0,0.5]
for i in range(0, N):
    print "Iteration %d" % (i)
    environment = omgtools.Environment(room={'shape': omgtools.Square(5.), 
                                    'position': [0, 0]})  # TODO: this shouldn't be a fixed size..

    cost = ComputeCostMatrix(WPi, WPf, Vi, Vf, environment)
       

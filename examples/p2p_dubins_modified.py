# This file is part of OMG-tools.
#
# OMG-tools -- Optimal Motion Generation-tools
# Copyright (C) 2016 Ruben Van Parys & Tim Mercy, KU Leuven.
# All rights reserved.
#
# OMG-tools is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA

from omgtools import *

# create vehicle
vehicle = DubinsModified(bounds={'vmax': .5, 'amax': 1., 'amin': -1., 'wmax': np.pi/3., 'wmin': -np.pi/3.}, # in rad/s
                 options={'substitution': True})
vehicle.define_knots(knot_intervals=5)  # choose lower amount of knot intervals

# Format for initial and final conditions: (x,y,\theta), (V, \dot(\theta) )
# TODO \dot(\theta) is not being considered. It is always set to 0.
# The following problem is feasible
vehicle.set_initial_conditions([1., 1., 0.],[0.5,0.])  # input orientation in rad
vehicle.set_terminal_conditions([2., 1., 0.],[0.5,0.]) 

# The following problem is infeasible but a solution is obtained
#vehicle.set_initial_conditions([1., 1., 0.],[0.5,0.])  # input orientation in rad
#vehicle.set_terminal_conditions([2., 1., 0.],[0.5,0.]) 

# The following problem obtains a wrong solution (most probably due to the tangent half angle substitution)
#vehicle.set_initial_conditions([1., 1., np.pi/2. + np.pi/4.],[0.5,0.])  # input orientation in rad
#vehicle.set_terminal_conditions([0., 1., -np.pi/2. - np.pi/4.],[0.5,0.])

# create environment
environment = Environment(room={'shape': Square(5.), 'position': [1.5, 1.5]})

# create a point-to-point problem
problem = Point2point(vehicle, environment, freeT=True)
# extra solver settings which may improve performance
problem.set_options({'solver_options':
    {'ipopt': {'ipopt.hessian_approximation': 'limited-memory'}}})

problem.init()

vehicle.problem = problem  # to plot error when using substitution

# create simulator
simulator = Simulator(problem)
problem.plot('scene')
vehicle.plot('input', knots=True, labels=['v (m/s)', 'w (rad/s)'])
vehicle.plot('state', knots=True, labels=['x (m)', 'y (m)', 'theta (rad)'])

# run it!
trajectories, signals = simulator.run()

T=trajectories['time'][-1][-1][-1]
state = trajectories['state'][0]
Vf=trajectories['input'][0][0][-1]
print(Vf)
time.sleep(5.5) 
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
vehicle = Holonomic()

vehicle.set_initial_conditions([0., 0.],[0.5, 0.])
vehicle.set_terminal_conditions([0., 1.], [0.5,0.])

# create environment
environment = Environment(room={'shape': Square(5.)})

# create a point-to-point problem
problem = Point2point(vehicle, environment, freeT=True)
problem.init()

# create simulator
simulator = Simulator(problem)
problem.plot('scene')
#vehicle.plot('input', knots=True, prediction=True, labels=['v_x (m/s)', 'v_y (m/s)'])
#vehicle.plot('dinput', knots=True, prediction=True, labels=['a_x (m/s)', 'a_y (m/s)'])

# run it!
trajectories, signals = simulator.run()

T=trajectories['time'][-1][-1][-1]
state = trajectories['state'][0]

diff = [np.linalg.norm( np.array([state[0][i+1],state[1][i+1]]) - np.array([state[0][i],state[1][i]])) for i in range(state.shape[1] - 1)]
traj_length = np.sum(diff)

print 'The trajectory time is: %.2f s' % T 
print 'The trajectory length is: %.2f m' % traj_length



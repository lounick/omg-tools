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
vehicle = Holonomic(shapes = Circle(0.4), bounds={'vxmin': -1, 'vymin': -1, 'vxmax': 1, 'vymax': 1, 'vmax':1,
							'axmin': -1, 'aymin': -1, 'axmax': 1, 'aymax': 1}, options={'syslimit':'norm_inf', 'velocity_weight': 100.})
veh_size = vehicle.shapes[0].radius

start = [-9,-9]
goal = [9,9] 
vehicle.set_initial_conditions(start)
vehicle.set_terminal_conditions(goal)

# create environment
environment = Environment(room={'shape': Square(20), 'position':[0,0], 'draw':True})

rect = Rectangle(width=4, height=4)
environment.add_obstacle(Obstacle({'position': [-6,-6]}, shape=rect))
environment.add_obstacle(Obstacle({'position': [-6,-0]}, shape=rect))
environment.add_obstacle(Obstacle({'position': [-6,6]}, shape=rect))
environment.add_obstacle(Obstacle({'position': [0,-6]}, shape=rect))
environment.add_obstacle(Obstacle({'position': [0,0]}, shape=rect))
environment.add_obstacle(Obstacle({'position': [0,6]}, shape=rect))
environment.add_obstacle(Obstacle({'position': [6,-6]}, shape=rect))
environment.add_obstacle(Obstacle({'position': [6,0]}, shape=rect))
environment.add_obstacle(Obstacle({'position': [6,6]}, shape=rect))

# add a dangerzone that moves over time, shows up at each crossroad when the vehicle comes in the neighbourhood
# in this example, the dangerzone is moved manually (i.e. 18 and 27 are chosen on purpose)
# normally, you would use the deployer for this, such that you can decide which DangerZone is closest to the vehicle
# but deployer doesn't have nice plotting, so I simulated the behaviour of deployer_example_dangerzone.py here
environment.add_danger_zone(DangerZone({'position': [-3, -3]}, shape=Rectangle(width=3, height=4),
                           bounds = {'vxmin': -0.5, 'vymin': -0.5, 'vxmax': 0.5, 'vymax': 0.5, 'vmax': 0.5},
                           simulation={'trajectories': {'position':{'time':[[18],[27]], 'values': [[6,0],[0,6]]}}}))

# make global planner
# [25,25] = number of cells in vertical and horizonal direction
# select 50,50 or 20,20 cells
globalplanner = AStarPlanner(environment, [50,50], start, goal, options={'veh_size': veh_size})

# make problem
# 'n_frames': number of frames to combine when searching for a trajectory
# 'check_moving_obs_ts': check in steps of ts seconds if a moving obstacle is inside the frame
# 'frame_type': 'corridor': creates corridors
  # 'scale_up_fine': tries to scale up the frame in small steps, leading to the largest possible corridor
  # 'l_shape': cuts off corridors, to obtain L-shapes, and minimize the influence of moving obstacles
# 'frame_type': 'shift': creates frames of fixed size, around the vehicle
  # 'frame_size': size of the shifted frame
options = {'frame_type': 'corridor', 'scale_up_fine': True, 'n_frames': 1, 'l_shape':False}
schedulerproblem = SchedulerProblem(vehicle, environment, globalplanner, options=options)
schedulerproblem.set_options({'solver_options': {'ipopt': {
                                                       #'ipopt.linear_solver': 'ma57',
                                                       # 'ipopt.hessian_approximation': 'limited-memory'
                                                       }}})

# simulate the problem
simulator = Simulator(schedulerproblem)

# define what you want to plot
schedulerproblem.plot('scene')
vehicle.plot('input', knots=True, prediction=True, labels=['v_x (m/s)', 'v_y (m/s)'])
vehicle.plot('dinput', knots=True, prediction=True, labels=['a_x (m/s^2)', 'a_y (m/s^2)'])

# run it!
simulator.run()

# schedulerproblem.save_movie('scene', format='gif', name='multiframe_pos2', number_of_frames=80, movie_time=8, axis=True)
# vehicle.save_movie('input', format='gif', name='multiframe_vel2', number_of_frames=80, movie_time=8, axis=True)
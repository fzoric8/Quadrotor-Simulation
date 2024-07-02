import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as Axes3D

from PathPlanning import RRTStar, Map
from TrajGen import trajGenerator, Helix_waypoints, Circle_waypoints, Linear_waypoints, gen_zigzag, interp_zigzag
from Quadrotor import QuadSim
import controller
np.random.seed(8)

# No obstacles for starters
obstacles = [[]]

# limits on map dimensions
bounds = np.array([0,100])
# create map with obstacles
#mapobs = Map(obstacles, bounds, dim = 3)

#plan a path from start to goal
start = np.array([0.8,0.2,0.1])
goal = np.array([0.4,0.4,2]) 
goal2 = np.array([0.4,0.6,0.8])

#rrt = RRTStar(start = start, goal = goal,
#              Map = mapobs, max_iter = 500,
#              goal_sample_rate = 0.1)
# RRT is used as a point (path) generator

pts = gen_zigzag(start[0], start[1], 10, 0.25)
pts_= interp_zigzag(pts, z_val=0.5)

#scale the waypoints to real dimensions
# waypoints = 0.02*waypoints

print(pts_)
#Generate trajectory through waypoints
traj = trajGenerator(pts_, max_vel = 3, gamma = 1e6)

#initialise simulation with given controller and trajectory
Tmax = traj.TS[-1]
des_state = traj.get_des_state
sim = QuadSim(controller,des_state,Tmax)

#create a figure
fig = plt.figure()
ax = Axes3D.Axes3D(fig)
ax.set_xlim((0,2))
ax.set_ylim((0,2))
ax.set_zlim((0,2))

#plot the waypoints and obstacles
#rrt.draw_path(ax, waypoints)
#mapobs.plotobs(ax, scale = 0.02)

#run simulation
print("Starting simulation!")
sim.run(ax)





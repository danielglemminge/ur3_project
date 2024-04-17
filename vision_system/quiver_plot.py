import matplotlib.pyplot as plt
import numpy as np


start = np.array([180, 225])
goal = np.array([480, 275])
theta_start_goal = np.arctan2(goal[1]-start[1], goal[0]-start[0])
x_lower = 165
x_upper = 500
y_lower = 170
y_upper = 370

x,y = np.meshgrid(np.arange(x_lower, x_upper, 20), np.arange(y_lower, y_upper, 20))
fig, ax = plt.subplots()
q = ax.quiver(x,y,1*np.cos(theta_start_goal),1*np.sin(theta_start_goal))

plt.plot(start[0], start[1], 'go', label='Start')
plt.plot(goal[0], goal[1], 'y*',markersize= 10, label='Goal')
plt.plot([start[0], goal[0]], [start[1], goal[1]], 'g--', label='Straight (Desired) Path')
plt.show()



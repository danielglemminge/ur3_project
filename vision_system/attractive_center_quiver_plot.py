import numpy as np
import matplotlib.pyplot as plt

def plot_quiver_meshgrid(pt0, pt1, x_range, y_range, scale=20):
    # Calculate the vector from pt0 to pt1
    vec = np.array(pt1) - np.array(pt0)
    
    # Calculate the normal vector (orthogonal to the line)
    normal_vec = np.array([-vec[1], vec[0]])
    
    # Normalize the normal vector
    normal_vec = normal_vec / np.linalg.norm(normal_vec)
    
    # Create meshgrid of points
    x, y = np.meshgrid(x_range, y_range)
    
    # Calculate vectors pointing orthogonally to the line at each point
    dx = x - pt0[0]
    dy = y - pt0[1]
    direction = dx * vec[1] - dy * vec[0]
    flip_direction = np.where(direction > 0, -1, 1)
    
    u = normal_vec[0] * -flip_direction
    v = normal_vec[1] * -flip_direction
    
    # Plot quiver meshgrid
    plt.quiver(x, y, u, v, scale=scale, pivot='tip')

# Example points
start = np.array([180, 225])
goal = np.array([480, 275])
theta_start_goal = np.arctan2(goal[1]-start[1], goal[0]-start[0])

# Define ranges for meshgrid
x_lower = 165
x_upper = 500
y_lower = 170
y_upper = 370
x_range = np.arange(x_lower, x_upper, 20)
y_range = np.arange(y_lower, y_upper, 20)
fig, ax = plt.subplots()
# Plot quiver meshgrid
plot_quiver_meshgrid(start, goal, x_range, y_range)

xc = ax.quiver(165, 230, 1*np.cos(theta_start_goal), 1*np.sin(theta_start_goal), scale=10, color='red')
yc = ax.quiver(165, 230, -1*np.sin(theta_start_goal), 1*np.cos(theta_start_goal), scale=10, color='green')


####
plt.plot(start[0], start[1], 'go', label='Start')
plt.plot(goal[0], goal[1], 'y*',markersize= 10, label='Goal')
plt.plot([start[0], goal[0]], [start[1], goal[1]], 'g--', label='Straight (Desired) Path')
plt.title('Attractive Force Vectors (Center)')
plt.legend()
plt.xlabel('X')
plt.ylabel('Y')
plt.show()

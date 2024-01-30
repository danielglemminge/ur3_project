import numpy as np
import matplotlib.pyplot as plt
from matplotlib import patches
import math

def distance(p1, p2):
    return np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)


def calculate_path(start, end, obstacle, num_points):
    x_start, y_start = start
    x_end, y_end = end
    x_obs, y_obs, obs_radius = obstacle
    gamma = 8

    
    path_x = np.linspace(x_start, x_end, num_points)
    path_y = np.linspace(y_start, y_end, num_points)
    
    for i in range(num_points):
        curr_point = [path_x[i],path_y[i]]
        dist_to_obstacle = distance(curr_point, [x_obs, y_obs])
        theta = math.atan2(curr_point[1]-y_obs, curr_point[0]-x_obs)
        dist_to_perimeter = obs_radius-dist_to_obstacle

        if dist_to_obstacle < obs_radius: # Point is inside obstacle
            #Gradient approach
            # path_x[i] += gamma*(dist_to_perimeter/dist_to_obstacle)*math.cos(theta)
            # path_y[i] += gamma*(dist_to_perimeter/dist_to_obstacle)*math.sin(theta)

            #Follow outline:
            path_x[i] += dist_to_perimeter*math.cos(theta)
            path_y[i] += dist_to_perimeter*math.sin(theta)




        else:
            pass
    coord_list = [i for i in zip(path_x, path_y)]
    return path_x, path_y, coord_list



def plot_path(start, end, obstacle, path_x, path_y):
    plt.figure(figsize=(8, 6))
    plt.plot([start[0], end[0]], [start[1], end[1]], 'b--', label='Original Path')
    plt.plot(path_x, path_y, 'r-', label='Adjusted Path')
    plt.plot(obstacle[0], obstacle[1], 'ro', label='Obstacle')
    plt.gca().add_patch(patches.Circle(obstacle[:2],obstacle[2], edgecolor='g', facecolor='none'))

    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Straight Line Path with Obstacle Avoidance')
    plt.legend()
    plt.grid(True)
    plt.show()

def main():
    start_point = (50, 50)
    end_point = (700, 400)
    obstacle = (370, 200, 80)  # (x, y, radius)
    num_points = 100
    
    path_x, path_y, _ = calculate_path(start_point, end_point, obstacle, num_points)
    plot_path(start_point, end_point, obstacle, path_x, path_y)

if __name__ == "__main__":
    main()

    


"""
def distance(p1, p2):
    return np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

def calculate_path(start, end, obstacle, num_points):
    x_start, y_start = start
    x_end, y_end = end
    x_obs, y_obs, obs_radius = obstacle
    
    path_x = np.linspace(x_start, x_end, num_points)
    path_y = np.linspace(y_start, y_end, num_points)
    
    for i in range(num_points):
        d_to_obs = distance((path_x[i], path_y[i]), obstacle[:2])
        if d_to_obs < obs_radius:
            gradient = (obs_radius - d_to_obs) / obs_radius
            path_x[i] += gradient * (x_obs - path_x[i])
            path_y[i] += gradient * (y_obs - path_y[i])
            
    return path_x, path_y

def plot_path(start, end, obstacle, path_x, path_y):
    plt.figure(figsize=(8, 6))
    plt.plot([start[0], end[0]], [start[1], end[1]], 'b--', label='Original Path')
    plt.plot(path_x, path_y, 'r-', label='Adjusted Path')
    plt.plot(obstacle[0], obstacle[1], 'ro', label='Obstacle')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Straight Line Path with Obstacle Avoidance')
    plt.legend()
    plt.grid(True)
    plt.show()

def main():
    start_point = (50, 50)
    end_point = (700, 400)
    obstacle = (350, 200, 20)  # (x, y, radius)
    num_points = 500
    
    path_x, path_y = calculate_path(start_point, end_point, obstacle, num_points)
    plot_path(start_point, end_point, obstacle, path_x, path_y)

if __name__ == "__main__":
    main()

"""

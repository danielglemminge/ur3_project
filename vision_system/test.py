import numpy as np
import matplotlib.pyplot as plt

def distance(p1, p2):
    return np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

def calculate_path(start, end, obstacle, num_points):
    x_start, y_start = start
    x_end, y_end = end
    x_obs, y_obs, obs_radius = obstacle
    
    path_x = np.linspace(x_start, x_end, num_points)
    path_y = np.linspace(y_start, y_end, num_points)
    
    for point in range(num_points):
        if distance([path_x,path_y], [x_obs, y_obs]) < obs_radius:
            print('innafor')

def plot_path(start, end, obstacle, path_x, path_y):
    plt.figure(figsize=(8, 6))
    plt.plot([start[0], end[0]], [start[1], end[1]], 'b--', label='Original Path')
    plt.plot(path_x, path_y, 'r-', label='Adjusted Path')
    plt.plot(obstacle[0], obstacle[1], 'ro', label='Obstacle', markersize=80)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Straight Line Path with Obstacle Avoidance')
    plt.legend()
    plt.grid(True)
    plt.show()

def main():
    start_point = (50, 50)
    end_point = (700, 400)
    obstacle = (350, 200, 100)  # (x, y, radius)
    num_points = 500
    
    path_x, path_y = calculate_path(start_point, end_point, obstacle, num_points)
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

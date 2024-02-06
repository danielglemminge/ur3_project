import numpy as np
import matplotlib.pyplot as plt
import copy
from matplotlib import patches
from numpy.linalg import norm

class PotentialFieldPlanner2:
    def __init__(self, start, goal, obstacles, k_att=0.04, k_rep=4, k_centerline=0.6, step_size=0.5, max_iters=50):
        self.start = start
        self.goal = goal
        self.obstacles = obstacles
        self.k_att = k_att
        self.k_rep = k_rep
        self.k_centerline = k_centerline
        self.step_size = step_size
        self.max_iters = max_iters

    def attractive_goal(self, position):
        theta_goal = np.arctan2(self.goal[1]-position[1], self.goal[0]-position[0]) # Angle between line from start-goal and x_axis
        d_goal = np.linalg.norm(self.start - self.goal) # Distance to goal
        attractive_goal_x = self.k_att * d_goal * np.cos(theta_goal)
        attractive_goal_y = self.k_att * d_goal * np.sin(theta_goal)
        
        return attractive_goal_x, attractive_goal_y
    
    def attractive_centerline(self, position):
        phi_centerline = np.arctan2(self.goal[1]-position[1], self.goal[0]-position[0])
        normal_distance_centerline = norm(np.cross(self.goal - self.start, self.start - position))/norm(self.goal - self.start)
        attractive_centerline_x = 0
        attractive_centerline_y = 0
        for obstacle in self.obstacles:
            radius = obstacle[2]
            dist_to_obstacle = np.linalg.norm(position - obstacle[:2])
            if dist_to_obstacle > radius: # Is the current point outside the obstacle?
                attractive_centerline_x = self.k_centerline * normal_distance_centerline *np.sin(phi_centerline) 
                attractive_centerline_y = self.k_centerline * normal_distance_centerline *np.cos(phi_centerline)
            else: # current point is inside obstacle, so centerline attraction is ignored
                pass
                
        return attractive_centerline_x, attractive_centerline_y

    def repulsive_obstacle(self, position):
        repulsive_obstacle_x = 0
        repulsive_obstacle_y = 0
        for obstacle in self.obstacles:
            radius = obstacle[2]
            distance = np.linalg.norm(position - obstacle[:2]) 
            if distance < radius:
                theta_obstacle = np.arctan2(obstacle[1]-position[1], obstacle[0]-position[0])
                repulsive_obstacle_x += self.k_rep * (radius**2/distance**2)*np.cos(theta_obstacle)
                repulsive_obstacle_y += self.k_rep * (radius**2/distance**2)*np.sin(theta_obstacle)
        return repulsive_obstacle_x, repulsive_obstacle_y

    def plan(self):
        current_position = self.start
        path = []
        path.append(current_position)

        for _ in range(self.max_iters):

            attractive_goal_x, attractive_goal_y = self.attractive_goal(current_position)
            repulsive_obstacle_x, repulsive_obstacle_y = self.repulsive_obstacle(current_position)
            attractive_centerline_x, attractive_centerline_y = self.attractive_centerline(current_position) 

            total_force_x = attractive_goal_x + attractive_centerline_x - repulsive_obstacle_x
            total_force_y = attractive_goal_y + attractive_centerline_y - repulsive_obstacle_y

            next_position_x = current_position[0] + self.step_size * total_force_x
            next_position_y = current_position[1] + self.step_size * total_force_y
            
            next_position = np.array([int(next_position_x), int(next_position_y)])

            if np.linalg.norm(next_position - self.goal) < self.step_size:
                path.append(self.goal)
                break

            path.append(next_position)
            current_position = next_position

        return np.array(path)

        
if __name__=="__main__":
        
    # Example usage:
    start = np.array([180, 180])
    goal = np.array([480, 360])
    #obstacles = [np.array([370, 280,20])]
    obstacles = [np.array([250, 225, 20]), np.array([350, 280, 20])]
    #obstacles = [np.array([300, 230,20]), np.array([300, 250, 20]), np.array([300, 270, 20])] # wall

    planner = PotentialFieldPlanner2(start, goal, obstacles)
    path = planner.plan()


    plt.plot(path[:, 0], path[:, 1], '-o', label='Planned Path')
    plt.plot(start[0], start[1], 'go', label='Start')
    plt.plot(goal[0], goal[1], 'ro', label='Goal')
    for obstacle in obstacles:
        plt.plot(obstacle[0], obstacle[1], 'ks', label='Obstacle')
        plt.gca().add_patch(patches.Circle(obstacle[:2],obstacle[2], edgecolor='g', facecolor='none'))
    plt.legend()
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Artificial Potential Field Path Planning 2.0')
    plt.grid(True)
    plt.show()


"""
To Do:
- plot force vectors

"""
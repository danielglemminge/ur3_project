import numpy as np
import matplotlib.pyplot as plt
import copy
from matplotlib import patches

class PotentialFieldPlanner:
    def __init__(self, start, goal, obstacles, k_att=0.4, k_rep=4, step_size=0.5, max_iters=50):
        self.start = start
        self.goal = goal
        self.obstacles = obstacles
        self.k_att = k_att
        self.k_rep = k_rep
        self.step_size = step_size
        self.max_iters = max_iters

    def attractive_potential(self, position):
        theta_goal = np.arctan2(self.goal[1]-position[1], self.goal[0]-position[0])
        # attractive_potential_x = (0.5 * self.k_att * np.linalg.norm(position - self.goal))*np.cos(theta_goal)
        # attractive_potential_y = (0.5 * self.k_att * np.linalg.norm(position - self.goal))*np.sin(theta_goal)
        attractive_potential_x = self.k_att * (np.linalg.norm(self.start - self.goal)/10)*np.cos(theta_goal)
        attractive_potential_y = self.k_att * (np.linalg.norm(self.start - self.goal)/10)*np.sin(theta_goal)
        
        return attractive_potential_x, attractive_potential_y

    def repulsive_potential(self, position):
        repulsive_force_x = 0
        repulsive_force_y = 0
        for obstacle in self.obstacles:
            radius = obstacle[2]
            distance = np.linalg.norm(position - obstacle[:2]) 
            if distance < radius:
                theta_obstacle = np.arctan2(obstacle[1]-position[1], obstacle[0]-position[0])
                repulsive_force_x += self.k_rep * (radius**2/distance**2)*np.cos(theta_obstacle)
                repulsive_force_y += self.k_rep * (radius**2/distance**2)*np.sin(theta_obstacle)
        return repulsive_force_x, repulsive_force_y

    def plan(self):
        current_position = self.start

        path = []
        path.append(current_position)

        for _ in range(self.max_iters):

            attractive_force_x, attractive_force_y = self.attractive_potential(current_position)
            repulsive_force_x, repulsive_force_y = self.repulsive_potential(current_position)

            total_force_x = attractive_force_x - repulsive_force_x
            total_force_y = attractive_force_y - repulsive_force_y

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

    planner = PotentialFieldPlanner(start, goal, obstacles)
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
    plt.title('Artificial Potential Field Path Planning')
    plt.grid(True)
    plt.show()

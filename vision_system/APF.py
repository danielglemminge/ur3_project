import numpy as np
import matplotlib.pyplot as plt
import copy

class PotentialFieldPlanner:
    def __init__(self, start, goal, obstacles, k_att=1, k_rep=1, rep_radius=5.0, step_size=1, max_iters=10):
        self.start = start
        self.goal = goal
        self.obstacles = obstacles
        self.k_att = k_att
        self.k_rep = k_rep
        self.rep_radius = rep_radius
        self.step_size = step_size
        self.max_iters = max_iters

    def attractive_potential(self, position):
        theta_goal = np.arctan2(goal[1]-position[1], goal[0]-position[0])
        attractive_potential_x = (0.5 * self.k_att * np.linalg.norm(position - self.goal))*np.cos(theta_goal)
        attractive_potential_y = (0.5 * self.k_att * np.linalg.norm(position - self.goal))*np.sin(theta_goal)
        return attractive_potential_x, attractive_potential_y

    def repulsive_potential(self, position):
        repulsive_force_x = 0
        repulsive_force_y = 0
        for obstacle in self.obstacles:
            distance = np.linalg.norm(position - obstacle) 
            if distance < self.rep_radius:
                theta_position_obstacle = np.arctan2(obstacle[1]-position[1], obstacle[0]-position[0])
                repulsive_force_x += -0.5 * (self.k_rep * ((1 / distance) - (1 / self.rep_radius)) **2)*np.cos(theta_position_obstacle)
                repulsive_force_y += -0.5 * (self.k_rep * ((1 / distance) - (1 / self.rep_radius)) **2)*np.sin(theta_position_obstacle)
        return repulsive_force_x, repulsive_force_y

    def plan(self):
        current_position = self.start
        path = [current_position]

        for _ in range(self.max_iters):
            attractive_force_x, attractive_force_y = self.attractive_potential(current_position)
            repulsive_force_x, repulsive_force_y = self.repulsive_potential(current_position)

            total_force_x = repulsive_force_x + attractive_force_x
            total_force_y = repulsive_force_y + attractive_force_y

            next_position_x = current_position[0] + self.step_size * total_force_x
            next_position_y = current_position[1] + self.step_size * total_force_y
            
            next_position = np.array([next_position_x, next_position_y])


            if np.linalg.norm(next_position - self.goal) < self.step_size:
                path.append(self.goal)
                break

            path.append(next_position)
            current_position = next_position

        return np.array(path)

# Example usage:
start = np.array([180, 180])
goal = np.array([480, 360])
# obstacles = [np.array([370, 280])]
obstacles = []

planner = PotentialFieldPlanner(start, goal, obstacles)
path = planner.plan()

plt.plot(path[:, 0], path[:, 1], '-o', label='Planned Path')
plt.plot(start[0], start[1], 'go', label='Start')
plt.plot(goal[0], goal[1], 'ro', label='Goal')
for obstacle in obstacles:
    plt.plot(obstacle[0], obstacle[1], 'ks', label='Obstacle')
plt.legend()
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Artificial Potential Field Path Planning')
plt.grid(True)
plt.show()

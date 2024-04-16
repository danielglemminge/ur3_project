import numpy as np
import matplotlib.pyplot as plt
import copy
from matplotlib import patches
import plottools


class PotentialFieldPlanner:
    def __init__(self, start, goal, obstacles, k_att=1, k_rep=1500, step_size=1, max_iters=1500, field_stretch = 20):
        self.start = start
        self.goal = goal
        self.obstacles = obstacles
        self.k_att = k_att
        self.k_rep = k_rep
        self.step_size = step_size
        self.max_iters = max_iters
        self.field_stretch = field_stretch

    def attractive_potential(self, position):
        theta_goal = np.arctan2(self.goal[1]-position[1], self.goal[0]-position[0])
        # attractive_potential_x = self.k_att * 0.009*(np.linalg.norm(self.start - self.goal))*np.cos(theta_goal)
        # attractive_potential_y = self.k_att * 0.009*(np.linalg.norm(self.start - self.goal))*np.sin(theta_goal)
        attractive_potential_x = self.k_att * np.cos(theta_goal)
        attractive_potential_y = self.k_att * np.sin(theta_goal)
        return attractive_potential_x, attractive_potential_y

    def repulsive_potential(self, position):
        repulsive_force_x = 0
        repulsive_force_y = 0
        for obstacle in self.obstacles:
            obs_radius = obstacle[2]
            distance = np.linalg.norm(position - obstacle[:2])
            eta_0 = obs_radius + self.field_stretch
            if distance < eta_0:
                print('inside')
                theta_obstacle = np.arctan2(obstacle[1]-position[1], obstacle[0]-position[0])
                # repulsive_force_x += self.k_rep/(distance-20)**2 * ((1/distance-20)-(1/radius-20))*np.cos(theta_obstacle)
                # repulsive_force_y += self.k_rep/(distance-20)**2 * ((1/distance-20)-(1/radius-20))*np.sin(theta_obstacle)
                repulsive_force_x += (self.k_rep/(distance-obs_radius)**2) * ((1/(distance-obs_radius))-(1/(eta_0)))*np.cos(theta_obstacle)
                repulsive_force_y += (self.k_rep/(distance-obs_radius)**2) * ((1/(distance-obs_radius))-(1/(eta_0)))*np.sin(theta_obstacle)
                print(repulsive_force_y)
        return repulsive_force_x, repulsive_force_y

    def plan(self):
        current_position = self.start

        path = []
        path.append(current_position)

        for i in range(self.max_iters):
            print(i)

            attractive_force_x, attractive_force_y = self.attractive_potential(current_position)
            repulsive_force_x, repulsive_force_y = self.repulsive_potential(current_position)
            print('att_x_y:', attractive_force_x, attractive_force_y)
            # print('rep_x_y:', repulsive_force_x, repulsive_force_y)
            total_force_x = attractive_force_x - repulsive_force_x
            total_force_y = attractive_force_y - repulsive_force_y

            # print('current:',current_position)
            # print('step_size:', self.step_size)

            next_position_x = current_position[0] + self.step_size * total_force_x
            next_position_y = current_position[1] + self.step_size * total_force_y
            # print('next pos:', next_position_x, next_position_y)

            next_position = np.array([next_position_x, next_position_y])

            if np.linalg.norm(next_position - self.goal) < self.step_size:
                path.append(self.goal)
                break

            path.append(next_position)
            current_position = next_position

        return np.array(path)

        
if __name__=="__main__":
        
    # Example usage:
    start = np.array([180, 225])
    goal = np.array([480, 275])
    #obstacles = [np.array([370, 280,20])]
    obstacles = [np.array([300, 240, 10])]
    # obstacles = [np.array([230, 230, 10]), np.array([430,260,10])]
    # obstacles = []
    # obstacles = [np.array([300, 230,20]), np.array([300, 250, 20])] # wall

    planner = PotentialFieldPlanner(start, goal, obstacles)
    path = planner.plan()



    plt.plot(path[:, 0], path[:, 1], '-o',linewidth=1,markersize=3, label='Planned Path')
    plt.plot(start[0], start[1], 'go', label='Start')
    plt.plot(goal[0], goal[1], 'y*',markersize= 10, label='Goal')
    straight_path = np.linspace(start, goal,200)
    plt.plot([start[0], goal[0]], [start[1], goal[1]], 'g--', label='Straight (Desired) Path')
    plt.plot([1000,1000], [1000,1000], 'r', label='Range of Influence')
    for obstacle in obstacles:
        plt.plot(obstacle[0], obstacle[1], 'ko', label='Obstacle')
        plt.gca().add_patch(patches.Circle(obstacle[:2],obstacle[2], edgecolor='black', facecolor='black'))
        plt.gca().add_patch(patches.Circle(obstacle[:2],obstacle[2]+planner.field_stretch, edgecolor='r', facecolor='none'))
    plt.legend()

    t=' $k_{a}=1$''\n $k_{r}= 1500$''\n $\eta_{0}= 20$'
    plt.text(165,340,t)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Standard APF Planner')
    plt.grid(False)
    plt.xlim(165, 500)
    plt.ylim(170, 370)
    plt.show()

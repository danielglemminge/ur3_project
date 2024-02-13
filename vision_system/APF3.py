import numpy as np
import matplotlib.pyplot as plt
import copy
from matplotlib import patches
from numpy.linalg import norm
import math


def closest_on_line(p1, p2, p3): # Returns a point[x,y] on the line from p1 to p2, that is closest to p3
        x1, y1 = p1 
        x2, y2 = p2
        x3, y3 = p3
        dx, dy = x2-x1, y2-y1
        det = dx*dx + dy*dy
        a = (dy*(y3-y1)+dx*(x3-x1))/det
        return [x1+a*dx, y1+a*dy]

class PotentialFieldPlanner2:
    def __init__(self, start, goal, obstacles, k_att=2, k_rep=15000, k_centerline=1, step_size=1, max_iters=300):
        self.start = start
        self.goal = goal
        self.obstacles = obstacles
        self.k_att = k_att
        self.k_rep = k_rep
        self.k_c = k_centerline
        self.step_size = step_size
        self.max_iters = max_iters


    def attractive_goal(self):
        theta_goal = np.arctan2(self.goal[1]-self.start[1], self.goal[0]-self.start[0]) # Angle between start position and goal
        attractive_goal_potential = 0
        attractive_goal_x = round(self.k_att * np.cos(theta_goal),5)
        attractive_goal_y = round(self.k_att * np.sin(theta_goal),5)
        
        return attractive_goal_x, attractive_goal_y, attractive_goal_potential
    
    
    def attractive_centerline(self, position):
        ref_point = closest_on_line(self.start, self.goal, position)
        phi_refpoint = np.arctan2(ref_point[1]-position[1], ref_point[0]-position[0])
        d_refpoint = np.linalg.norm(position - ref_point)

        if len(self.obstacles) > 0:
            for obstacle in self.obstacles:
                dist_to_obstacle = np.linalg.norm(position - obstacle[:2])
                obstacle_radius = obstacle[2]
                if dist_to_obstacle > obstacle_radius:
                    centerline_potential = round((0.5*self.k_c*(d_refpoint)**2),5) # 0.5*K_c*d**2
                    force_centerline_x = round(self.k_c**d_refpoint * (np.cos(phi_refpoint)),5)
                    force_centerline_y = round(self.k_c**d_refpoint * (np.sin(phi_refpoint)),5)
                else:
                    centerline_potential = 0
                    force_centerline_x = 0
                    force_centerline_y = 0
        else: 
            centerline_potential = round((0.5*self.k_c*(d_refpoint)**2),5) # 0.5*K_c*d**2
            force_centerline_x = round(self.k_c**d_refpoint * (np.cps(phi_refpoint)),5)
            force_centerline_y = round(self.k_c**d_refpoint * (np.sin(phi_refpoint)),5)
        
        return force_centerline_x, force_centerline_y, centerline_potential
    

    def repulsive_obstacle(self, position):
        if len(self.obstacles)>=1:    
            repulsive_force_x = 0
            repulsive_force_y = 0
            repulsive_potential = 0
            for obstacle in self.obstacles:
                radius = obstacle[2]
                distance = np.linalg.norm(position - obstacle[:2])
                theta_obstacle = np.arctan2(obstacle[1]-position[1], obstacle[0]-position[0])
                if distance <= radius:
                    repulsive_potential += 0.5*self.k_rep*((1/distance)-(1/radius))**2
                    repulsive_force = -(self.k_rep/distance**2)*((1/distance)-(1/(radius)))
                    repulsive_force_x += round(repulsive_force * np.cos(theta_obstacle),5)
                    repulsive_force_y += round(repulsive_force * np.sin(theta_obstacle),5)
                    print(repulsive_force)
                   
                else:
                    repulsive_potential += 0
                    repulsive_force_x += 0
                    repulsive_force_y += 0
                    print(repulsive_force_x)
        else:
            repulsive_potential = 0
            repulsive_force_x = 0
            repulsive_force_y = 0
            print(repulsive_force_x)
        return repulsive_force_x, repulsive_force_y, repulsive_potential

    def plan(self):
        current_position = self.start
        path = []
        path.append(current_position)

        for _ in range(self.max_iters):

            attractive_goal_x, attractive_goal_y, goal_potential = self.attractive_goal()
            repulsive_obstacle_x, repulsive_obstacle_y, repulsive_potential = self.repulsive_obstacle(current_position)
            attractive_centerline_x, attractive_centerline_y, centerline_potential = self.attractive_centerline(current_position)
            

            total_force_x = attractive_goal_x + attractive_centerline_x + repulsive_obstacle_x
            total_force_y = attractive_goal_y + attractive_centerline_y + repulsive_obstacle_y

            next_position_x = current_position[0] + self.step_size * total_force_x
            next_position_y = current_position[1] + self.step_size * total_force_y
            next_position = np.array([next_position_x, next_position_y])
            next_position = np.array([next_position_x, next_position_y]) # For precise calculations

            if np.linalg.norm(next_position - self.goal) < self.step_size:
                path.append(self.goal)
                break

            path.append(next_position)
            current_position = next_position

        return np.array(path)
    

    def plotTerrain(self):
        x = np.arange(self.start[0], self.goal[0],1)
        y = np.arange(self.start[0], self.goal[0],1)

        X, Y = np.meshgrid(x,y)

        total_force_x = np.zeros_like(X)
        total_force_y = np.zeros_like(Y)

        for i in range(0, len(x), 5):
            for j in range(0, len(y), 5):
                print(i,j)
                _, _, goal_potential = self.attractive_goal()
                attractive_centerline_x, attractive_centerline_y = self.attractive_centerline(np.array([i,j]))
                repulsive_obstacle_x, repulsive_obstacle_y = self.repulsive_obstacle(np.array([i,j]))
                #print(attractive_centerline_x, attractive_goal_x, repulsive_obstacle_x)
                #total_potential[i][j] = goal_potential
        
        fig, ax = plt.subplots(figsize = (abs(self.goal[0]-self.start[0]), abs(self.goal[1]-self.start[1])))
        ax.quiver(X[::5], Y[::5], total_force_x[::5], total_force_y[::5])
        ax.add_patch(plt.Circle(self.start, 2, color = 'g'))
        ax.add_patch(plt.Circle(self.goal, 2, color = 'm'))
        for obstacle in self.obstacles:
            plt.plot(obstacle[0], obstacle[1], 'ks', label='Obstacle')
            plt.gca().add_patch(patches.Circle(obstacle[:2],obstacle[2], edgecolor='g', facecolor='none'))
        ax.set_title('Combined Potential when Goal and Obstacle are different ')
        plt.show()



        

        pass

        
if __name__=="__main__":
        
    # Example usage:
    start = np.array([180, 180])
    goal = np.array([480, 350])
    #obstacles = [np.array([370, 280,20])]
    obstacles = [np.array([250, 225, 20]), np.array([400, 300, 40])]
    # obstacles = [] 

    planner = PotentialFieldPlanner2(start, goal, obstacles)
    path = planner.plan()


    plt.plot(path[:, 0], path[:, 1], '-o', label='Planned Path')
    plt.plot(start[0], start[1], 'go', label='Start')
    plt.plot(goal[0], goal[1], 'ro', label='Goal')
    straight_path = np.linspace(start, goal,200)
    plt.plot([start[0], goal[0]], [start[1], goal[1]], 'b--', label='Straight Line')
    for obstacle in obstacles:
        plt.plot(obstacle[0], obstacle[1], 'ks', label='Obstacle')
        plt.gca().add_patch(patches.Circle(obstacle[:2],obstacle[2], edgecolor='g', facecolor='none'))
    plt.legend()
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Artificial Potential Field Path Planning 2.0')
    plt.grid(True)
    plt.xlim(165, 500)
    plt.ylim(180, 360)
    plt.show()

    #planner.plotTerrain()


"""
To Do:
- plot force vectors

"""
import numpy as np
import matplotlib.pyplot as plt
import copy
from matplotlib import patches
from numpy.linalg import norm
import math
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import cv2


def closest_on_line(p1, p2, p3): # Returns a point[x,y] on the line from p1 to p2, that is closest to p3
        x1, y1 = p1 
        x2, y2 = p2
        x3, y3 = p3
        dx, dy = x2-x1, y2-y1
        det = dx*dx + dy*dy
        a = (dy*(y3-y1)+dx*(x3-x1))/det
        return [x1+a*dx, y1+a*dy]

# Working params for planner:
# k_att=3, k_rep=40000, k_centerline=0.3, step_size=1, max_iters=300
class PotentialFieldPlanner4:
    def __init__(self, start, goal, obstacles_with_centers_zip, k_att=3, k_rep=20000, k_centerline=0.5, step_size=0.5, goal_threshold=2, max_iters=400):
        self.start = start
        self.goal = goal
        self.obstacles_with_centers_zip = obstacles_with_centers_zip
        self.k_att = k_att
        self.k_rep = k_rep
        self.k_c = k_centerline
        self.step_size = step_size
        self.max_iters = max_iters
        self.goal_threshold = goal_threshold
        self.obstacles_present = len(obstacles_with_centers_zip) > 0 # True if obstacles is present, else False

    def attractive_goal(self):
        theta_goal = np.arctan2(self.goal[1]-self.start[1], self.goal[0]-self.start[0]) # Angle between start position and goal
        goal_potential = 0
        force_goal_x = round(self.k_att * (np.cos(theta_goal)),5)
        force_goal_y = round(self.k_att * (np.sin(theta_goal)),5)
        
        return force_goal_x, force_goal_y, goal_potential
    
    
    def attractive_centerline(self, position):
        ref_point = closest_on_line(self.start, self.goal, position)
        phi_refpoint = np.arctan2(ref_point[1]-position[1], ref_point[0]-position[0])
        d_refpoint = np.linalg.norm(position - ref_point)
        exponent = 2
        if self.obstacles_present:
            for cnt, center in self.obstacles_with_centers_zip:
                inside_test = cv2.pointPolygonTest(cnt, position,False)
                if inside_test == True: # 1:point inside, 0:point on contour, -1:point outside
                     
                     
                
        else: 
            centerline_potential = round((self.k_c/exponent*(d_refpoint)**exponent),5) # 0.5*K_c*d**2
            if centerline_potential > 3:
                        centerline_potential = 3
            force_centerline_x = round(self.k_c*(d_refpoint)**(exponent-1) * (np.cos(phi_refpoint)),5)
            force_centerline_y = round(self.k_c*(d_refpoint)**(exponent-1) * (np.sin(phi_refpoint)),5)
        
        return force_centerline_x, force_centerline_y, centerline_potential
    

    def repulsive_obstacle(self, position):
        repulsive_force_x = 0
        repulsive_force_y = 0
        repulsive_potential = 0
        return repulsive_force_x, repulsive_force_y, repulsive_potential

    def plan(self):
        current_position = self.start
        path = []
        path.append(current_position)

        for _ in range(self.max_iters):

            attractive_goal_x, attractive_goal_y, _ = self.attractive_goal()
            repulsive_obstacle_x, repulsive_obstacle_y, _ = self.repulsive_obstacle(current_position)
            attractive_centerline_x, attractive_centerline_y, _ = self.attractive_centerline(current_position)
            

            total_force_x = attractive_goal_x + attractive_centerline_x + repulsive_obstacle_x
            total_force_y = attractive_goal_y + attractive_centerline_y + repulsive_obstacle_y

            next_position_x = current_position[0] + self.step_size * total_force_x
            next_position_y = current_position[1] + self.step_size * total_force_y
            next_position = np.array([next_position_x, next_position_y])
            next_position = np.array([next_position_x, next_position_y]) # For precise calculations

            if np.linalg.norm(next_position - self.goal) < self.goal_threshold:
                path.append(self.goal)
                break

            path.append(next_position)
            current_position = next_position

        return np.array(path)


        
if __name__=="__main__":
        
    # Example usage:
    start = np.array([194, 477])
    goal = np.array([618, 349])
    #obstacles = [np.array([370, 280,20])]
    obstacles = [np.array([308,431,20]), np.array([445, 405, 10])]
    # obstacles = [] 

    planner = PotentialFieldPlanner4(start, goal, obstacles)
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
    plt.title('Artificial Potential Field Path Planning 3.0')
    plt.grid(True)
    plt.show()

    #planner.plotTerrain()


"""
To Do:
- plot force vectors
- small change


"""
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
        return [round(x1+a*dx), round(y1+a*dy)]

def move_along_line(A, B, distance):
    # Calculate the direction vector from A to B
    direction_vector = (B[0] - A[0], B[1] - A[1])
    
    # Calculate the magnitude of the direction vector
    magnitude = math.sqrt(direction_vector[0]**2 + direction_vector[1]**2)
    
    # Normalize the direction vector to get the unit vector
    unit_vector = (direction_vector[0] / magnitude, direction_vector[1] / magnitude)
    
    # Calculate the new point's coordinates by moving along the unit vector
    new_point = (A[0] + unit_vector[0] * distance, A[1] + unit_vector[1] * distance)
    
    return new_point




# Working params for planner:
# k_att=3, k_rep=40000, k_centerline=0.3, step_size=1, max_iters=300
class PotentialFieldPlanner4:
    def __init__(self, start, goal, hull_list, mass_center_list, k_att=2, k_rep=1, k_centerline=1, obstacle_scale = 1.2, step_size=1, goal_threshold=2, max_iters=1000):
        self.start = start
        self.goal = goal
        self.theta_start_goal = np.arctan2(goal[1]-start[1], goal[0]-start[0])
        self.hull_list = hull_list
        self.mass_center_list = mass_center_list
        self.k_att = k_att
        self.k_rep = k_rep
        self.k_c = k_centerline
        self.step_size = step_size
        self.max_iters = max_iters
        self.obstacle_scale = obstacle_scale
        self.goal_threshold = goal_threshold
        self.obstacles_present = len(hull_list) >= 1 # True if obstacles is present, else False
        #self.obstacles_present = False #For turning off obstacle avoidance


    def attractive_goal(self, position):
        d_goal = np.linalg.norm(position - self.goal)
        goal_potential = round(self.k_att*d_goal,1)
        force_goal = self.k_att
        force_goal_x = force_goal * (np.cos(self.theta_start_goal))
        force_goal_y = force_goal * (np.sin(self.theta_start_goal))
        return round(force_goal_x, 2), round(force_goal_y, 1), round(goal_potential, 1)
    
    
    def attractive_centerline(self, position):
        ref_point = closest_on_line(self.start, self.goal, position)
        print(ref_point)
        phi_refpoint = np.arctan2(ref_point[1]-position[1], ref_point[0]-position[0])
        d_refpoint = np.linalg.norm(position - ref_point)
        if self.obstacles_present:
            for hull in self.hull_list:
                inside_check = cv2.pointPolygonTest(hull, np.ndarray.tolist(position),False)
                if inside_check == -1: # 1:point inside, 0:point on contour, -1:point outside
                     centerline_potential = self.k_c * d_refpoint
                     force_centerline = self.k_c
                     force_centerline_x = force_centerline * np.cos(phi_refpoint)
                     force_centerline_y = force_centerline * np.sin(phi_refpoint)
                else:
                    centerline_potential = 0
                    force_centerline = 0
                    force_centerline_x = force_centerline * np.cos(phi_refpoint)
                    force_centerline_y = force_centerline * np.sin(phi_refpoint)
        else:
            centerline_potential = self.k_c*d_refpoint
            force_centerline = self.k_c
            force_centerline_x = force_centerline * np.cos(phi_refpoint)
            force_centerline_y = force_centerline * np.sin(phi_refpoint)
        
        return round(force_centerline_x, 2), round(force_centerline_y, 2), round(centerline_potential, 2)
    

    def repulsive_obstacle(self, position):
        if self.obstacles_present:
            repulsive_potential = 0
            repulsive_force = 0
            repulsive_force_x = 0
            repulsive_force_y = 0
            for cnt, center in zip(self.hull_list, self.mass_center_list):
                
                inside_check = cv2.pointPolygonTest(cnt, np.ndarray.tolist(position), False)
                if inside_check == 1: # 1:point inside, 0:point on contour, -1:point outside
                    # Create an imaginary line paralel to the start-stop line through the center of mass of the contour, then find the orthogonal point to position
                    refpoint = closest_on_line(center, [center[0]+50*np.cos(self.theta_start_goal), center[1]+50*np.sin(self.theta_start_goal)], position)
                    d_refpoint = np.linalg.norm(refpoint - position)
                    point_inside_check = 1
                    step = d_refpoint + 1
                    while point_inside_check >= 0:
                        edge_point = move_along_line(refpoint, position, step) # move from orthogonal point along center of mass line though current position until outside contour
                        point_inside_check = cv2.pointPolygonTest(cnt, [edge_point[0], edge_point[1]], False)
                        step += 1
                    d_contour = np.linalg.norm(position - edge_point)

                    theta_repulsive = np.arctan2(refpoint[1] - edge_point[1], refpoint[0] - edge_point[0])
                    theta_repulsive2 = np.arctan2(edge_point[1]- refpoint[1] , edge_point[0] - refpoint[0])
                    # print('Theta_start_goal',np.rad2deg(self.theta_start_goal))
                    print('Theta_repulsive',np.rad2deg(theta_repulsive))
                    print('Theta_repulsive2',np.rad2deg(theta_repulsive2))
                    # print('Dot product', np.dot(self.theta_start_goal, theta_repulsive))
                    
                
                    repulsive_potential += 0.5 * self.k_rep * d_contour**2
                    repulsive_force += self.k_rep*d_contour
                    repulsive_force_x += repulsive_force * np.cos(theta_repulsive2)
                    repulsive_force_y += repulsive_force * np.sin(theta_repulsive2)
                else:
                    repulsive_potential += 0
                    repulsive_force += 0
                    repulsive_force_x += 0
                    repulsive_force_y += 0  
        else:
            repulsive_potential = 0
            repulsive_force = 0
            repulsive_force_x = 0
            repulsive_force_y = 0

        return round(repulsive_force_x, 2), round(repulsive_force_y, 2), round(repulsive_potential, 2)

    def plan(self):
        current_position = self.start
        path = []
        path.append(current_position)
        counter = 1
        for _ in range(self.max_iters):
            print(counter)
            counter += 1

            attractive_goal_x, attractive_goal_y, _ = self.attractive_goal(current_position)
            repulsive_obstacle_x, repulsive_obstacle_y, _ = self.repulsive_obstacle(current_position)
            attractive_centerline_x, attractive_centerline_y, _ = self.attractive_centerline(current_position)
            

            total_force_x = attractive_goal_x + attractive_centerline_x + repulsive_obstacle_x
            total_force_y = attractive_goal_y + attractive_centerline_y + repulsive_obstacle_y

            next_position_x = current_position[0] + self.step_size * total_force_x
            next_position_y = current_position[1] + self.step_size * total_force_y
            next_position = np.array([next_position_x, next_position_y])
            next_position = np.array([next_position_x, next_position_y]) # For precise calculations

            if np.linalg.norm(next_position - self.goal) < self.goal_threshold:
                print('within goal thresh')
                path.append(self.goal)
                break

            path.append(next_position)
            current_position = next_position
            print('CURRENT POSITION: ',current_position)

        return np.array(path)


        
if __name__=="__main__":
        
    # Example usage:
    start = np.array([194, 477])
    goal = np.array([618, 349])
    #obstacles = [np.array([370, 280,20])]
    #obstacles = [np.array([308,431,20]), np.array([445, 405, 10])]
    #obstacles = [] 

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
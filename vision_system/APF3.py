import numpy as np
import matplotlib.pyplot as plt
import copy
from matplotlib import patches
from numpy.linalg import norm
import math
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm


def closest_on_line(p1, p2, p3): # Returns a point[x,y] on the line from p1 to p2, that is closest to p3
        x1, y1 = p1 
        x2, y2 = p2
        x3, y3 = p3
        dx, dy = x2-x1, y2-y1
        det = dx*dx + dy*dy
        a = (dy*(y3-y1)+dx*(x3-x1))/det
        return [x1+a*dx, y1+a*dy]

class PotentialFieldPlanner2:
    def __init__(self, start, goal, obstacles, k_att=3, k_rep=15000, k_centerline=0.5, step_size=1, max_iters=300):
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
        goal_potential = 0
        force_goal_x = round(self.k_att * (np.cos(theta_goal)),5)
        force_goal_y = round(self.k_att * (np.sin(theta_goal)),5)
        
        return force_goal_x, force_goal_y, goal_potential
    
    
    def attractive_centerline(self, position):
        ref_point = closest_on_line(self.start, self.goal, position)
        phi_refpoint = np.arctan2(ref_point[1]-position[1], ref_point[0]-position[0])
        d_refpoint = np.linalg.norm(position - ref_point)

        if len(self.obstacles) > 0:
            for obstacle in self.obstacles:
                dist_to_obstacle = np.linalg.norm(position - obstacle[:2])
                obstacle_radius = obstacle[2]
                if dist_to_obstacle > obstacle_radius:
                    exponent = 2
                    centerline_potential = round((self.k_c/exponent*(d_refpoint)**exponent),5) # 0.5*K_c*d**2
                    force_centerline_x = round(self.k_c*(d_refpoint)**(exponent-1) * (np.cos(phi_refpoint)),5)
                    force_centerline_y = round(self.k_c*(d_refpoint)**(exponent-1) * (np.sin(phi_refpoint)),5)
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
        # http://www.diag.uniroma1.it/oriolo/amr/slides/MotionPlanning3_Slides.pdf
        
        if len(self.obstacles)>=1:    
            repulsive_force_x = 0
            repulsive_force_y = 0
            repulsive_potential = 0
            for obstacle in self.obstacles:
                radius = obstacle[2]
                distance = np.linalg.norm(position - obstacle[:2])
                theta_obstacle = np.arctan2(obstacle[1]-position[1], obstacle[0]-position[0])
                if distance <= radius:
                    if distance > 10:
                            
                        exponent = 2 # if exponent=2->gain should be around 15000, if exponent=1, gain should be around 150
                        repulsive_potential += (self.k_rep/exponent)*((1/distance)-(1/radius))**exponent
                        repulsive_force = -(self.k_rep/distance**2)*((1/distance)-(1/(radius)))**(exponent-1)
                        repulsive_force_x += round(repulsive_force * np.cos(theta_obstacle),5)
                        repulsive_force_y += round(repulsive_force * np.sin(theta_obstacle),5)
                    else:
                        repulsive_potential += 200 # cutoff to avoid infinitely scaling function.
                else:
                    repulsive_potential += 0
                    repulsive_force_x += 0
                    repulsive_force_y += 0
        else:
            repulsive_potential = 0
            repulsive_force_x = 0
            repulsive_force_y = 0
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

            if np.linalg.norm(next_position - self.goal) < self.step_size:
                path.append(self.goal)
                break

            path.append(next_position)
            current_position = next_position

        return np.array(path)
    

    def plotTerrain(self):
        x = np.arange(0, self.goal[0],1)
        y = np.arange(0, self.goal[0],1)

        X, Y = np.meshgrid(x,y)

        Z = np.zeros_like(X)
    
        for i in range(0, len(x), 1):
            for j in range(0, len(y), 1):
                current_position = np.array([i,j])
                _, _, goal_potential = self.attractive_goal()
                _,_, centerline_potential = self.attractive_centerline(current_position)
                _, _, obstacle_potential = self.repulsive_obstacle(current_position)
                Z[i,j] = 0*goal_potential + centerline_potential + obstacle_potential 
                    
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot_surface(X, Y, Z, cmap='viridis')
        
        

        # Customize labels
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Total potential')
        
        plt.show()


        
if __name__=="__main__":
        
    # Example usage:
    start = np.array([180, 180])
    goal = np.array([480, 350])
    #obstacles = [np.array([370, 280,20])]
    obstacles = [np.array([250, 225, 20]), np.array([400, 300, 20])]
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

    planner.plotTerrain()


"""
To Do:
- plot force vectors

"""
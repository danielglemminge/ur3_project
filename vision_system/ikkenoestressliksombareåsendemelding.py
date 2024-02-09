import numpy as np



import numpy as np
import matplotlib.pyplot as plt

# Given points
point1 = np.array([5, 1])
point2 = np.array([2, 3])

# Calculate the distance between point1 and point2
distance = np.linalg.norm(point2 - point1)

# Calculate the angle between point1 and point2
angle = np.arctan2(point2[1] - point1[1], point2[0] - point1[0])

# Determine the amount to move point1 towards point2
d = 2  # specify how much you want to move point1 towards point2

# Calculate the new coordinates for point1
new_x = point1[0] + d * np.cos(angle)
new_y = point1[1] + d * np.sin(angle)

# Update point1 with the new coordinates
new_point1 = np.array([new_x, new_y])

# Plotting
plt.plot([point1[0], point2[0]], [point1[1], point2[1]], 'ro-')  # Plot line between point1 and point2
plt.plot(point1[0], point1[1], 'bo')  # Plot point1
plt.plot(new_point1[0], new_point1[1], 'go')  # Plot new_point1 after moving
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Move point1 towards point2')
plt.axis('equal')
plt.grid(True)
plt.legend(['Line between point1 and point2', 'point1', 'New point1'])
plt.show()

import numpy as np
import cv2
import random
import copy

black_spot_mask_png = cv2.imread('/home/daniel/catkin_ws/src/ur3_project/vision_system/input_images/1st_melanin_spot.png')
black_spot_mask = cv2.cvtColor(black_spot_mask_png, cv2.COLOR_BGR2GRAY) # Binary transformation

# Find contour
contours, _ = cv2.findContours(black_spot_mask,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
print('Number of spots: ', len(contours[0]))
black_spot_list = []
for contour in contours:
    (x,y),radius = cv2.minEnclosingCircle(contour)
    x_y_radius = np.array([int(x), int(y), int(radius)])
    black_spot_list.append(x_y_radius)
    color = (random.randint(0,256), random.randint(0,256), random.randint(0,256))
    cv2.circle(black_spot_mask_png, x_y_radius[:2], x_y_radius[2], color, thickness=2)

cv2.imshow('',black_spot_mask_png)
cv2.waitKey(0)
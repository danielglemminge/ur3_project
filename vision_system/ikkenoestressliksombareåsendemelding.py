from __future__ import print_function
import cv2 as cv
import numpy as np
import argparse
import random as rng
import cv2
import copy
from itertools import zip_longest


def scale_contour(contours, scale):
    contours_scaled = []
    for cnt in contours:    
        M = cv2.moments(cnt)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])

        cnt_norm = cnt - [cx, cy]
        cnt_scaled = cnt_norm * scale
        cnt_scaled = cnt_scaled + [cx, cy]
        cnt_scaled = cnt_scaled.astype(np.int32)
        contours_scaled.append(cnt_scaled)

    return contours_scaled

convexhullinput = '/home/daniel/catkin_ws/src/ur3_project/vision_system/input_images/convex_hull_input.jpg'
convexhullinput2 = '/home/daniel/catkin_ws/src/ur3_project/vision_system/images_michael/melanin_mask_f1_extreme.jpg'


melanin_mask = cv2.imread(convexhullinput2) # Read
im_draw = copy.deepcopy(melanin_mask)

melanin_mask = cv2.cvtColor(melanin_mask, cv2.COLOR_BGR2GRAY) # Grayscale
ret,thresh = cv2.threshold(melanin_mask, 120,255,cv2.THRESH_BINARY) # Binary transformation
cnt, _ = cv2.findContours(thresh,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
print('Number of spots: ', len(cnt))
hull_list = [cv2.convexHull(c) for c in cnt] # find the convexhull of the contour



hulls_scaled = scale_contour(hull_list, 1.2)



cv2.drawContours(im_draw, hulls_scaled, -1, (0,255,0), 2)
# display the image
cv2.imshow("Image", im_draw)
cv2.waitKey(0)

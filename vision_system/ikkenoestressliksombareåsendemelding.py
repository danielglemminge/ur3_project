from __future__ import print_function
import cv2 as cv
import numpy as np
import argparse
import random as rng
import cv2
import copy

convexhullinput = '/home/daniel/catkin_ws/src/ur3_project/vision_system/input_images/convex_hull_input.jpg'


melanin_mask = cv2.imread(convexhullinput) # Read
melanin_mask = cv2.resize(melanin_mask, (0, 0), fx = 0.5, fy = 0.5)
im_draw = copy.deepcopy(melanin_mask)

melanin_mask = cv2.cvtColor(melanin_mask, cv2.COLOR_BGR2GRAY) # Grayscale
ret,thresh = cv2.threshold(melanin_mask, 120,255,cv2.THRESH_BINARY) # Binary transformation
cnt, _ = cv2.findContours(thresh,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
hull_list = [cv2.convexHull(c) for c in cnt] # find the convexhull of the contour

for h in hull_list:
   #hull = cv2.convexHull(c)
   # calculate moments for each contour
   M = cv2.moments(h)
 
   # calculate x,y coordinate of center
   cX = int(M["m10"] / M["m00"])
   cY = int(M["m01"] / M["m00"])
   print(cX,cY)
   cv2.circle(im_draw, (cX, cY), 5, (0, 0, 255), -1)
   cv2.putText(im_draw, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
 
   # display the image
   cv2.imshow("Image", im_draw)
   cv2.waitKey(0)

point= (50,50)


result = cv2.pointPolygonTest(hull_list[0], point,False)

if result == 1:
    print("The point is inside the contour.")
elif result == 0:
    print("The point is on the contour.")
else:
    print("The point is outside the contour.")

print(result)



# for contour in cnt:
#         hull = cv2.convexHull(contour)
#         cv2.drawContours(im_draw, [hull], -1, (0,0,255), thickness=2)



# cv2.imshow('input',im_draw)
# cv2.waitKey()

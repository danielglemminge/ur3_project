from __future__ import print_function
import cv2 as cv
import numpy as np
import argparse
import random as rng
import cv2
import copy
from itertools import zip_longest

convexhullinput = '/home/daniel/catkin_ws/src/ur3_project/vision_system/input_images/convex_hull_input.jpg'
convexhullinput2 = '/home/daniel/catkin_ws/src/ur3_project/vision_system/images_michael/melanin_mask_f5.jpg'


melanin_mask = cv2.imread(convexhullinput2) # Read
#melanin_mask = cv2.resize(melanin_mask, (0, 0), fx = 0.5, fy = 0.5)
im_draw = copy.deepcopy(melanin_mask)

melanin_mask = cv2.cvtColor(melanin_mask, cv2.COLOR_BGR2GRAY) # Grayscale
ret,thresh = cv2.threshold(melanin_mask, 120,255,cv2.THRESH_BINARY) # Binary transformation
cnt, _ = cv2.findContours(thresh,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
print('Number of spots: ', len(cnt))
hull_list = [cv2.convexHull(c) for c in cnt] # find the convexhull of the contour
centroid_list = []

for h in hull_list:
   # calculate moments for each contour
   M = cv2.moments(h)
   
 
   # calculate x,y coordinate of center
   cX = int(M["m10"] / M["m00"])
   cY = int(M["m01"] / M["m00"])
   centroid_list.append([cX, cY])
   cv2.circle(im_draw, (cX,cY), 5, (0, 0, 255), -1)


cnt_cm_zip = zip(hull_list, centroid_list)







for contour_list, cm in cnt_cm_zip:
    print(len(contour_list))
    print(contour_list)
    cv2.drawContours(im_draw, contour_list, -1, (255,0,0), 2)
    cv2.circle(im_draw, cm, 1, (0,0,255),2)                    
    cv2.imshow('im_black_spot', im_draw)
    testpoint = contour_list[-1]





cv2.drawContours(im_draw, hull_list, -1, (0,255,0), 2)
# display the image
cv2.imshow("Image", im_draw)
cv2.waitKey(0)

point= (300,430)


result = cv2.pointPolygonTest(hull_list[0], point, False)

if result == True:
    print("The point is inswwwwwqide the contour.")
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

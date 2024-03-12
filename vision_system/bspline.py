

import numpy as np
from scipy.interpolate import splprep, splev
import matplotlib.pyplot as plt
import cv2 as cv
 

path_to_input = '/home/daniel/catkin_ws/src/ur3_project/vision_system/'

for f in '12345':
    img = cv.imread(path_to_input+'images_michael/binary_mask_f'+f+'.jpg') # Read
    belly_mask = cv.cvtColor(img, cv.COLOR_BGR2GRAY) # Grayscale
    ret,thresh = cv.threshold(belly_mask, 120,255,cv.THRESH_BINARY) # Binary transformation

    contours,hierarchy = cv.findContours(thresh, 1, 2)
    cnt = contours[0]
    M = cv.moments(cnt)

    rows,cols = img.shape[:2]
    [vx,vy,x,y] = cv.fitLine(cnt, cv.DIST_L2,0,0.01,0.01)

    pt0 = np.array([x-(50*vx), y-(50*vy)])
    pt1 = np.array([x+(50*vx), y+(50*vy)])
    print('sssss',pt0, pt1)

    inside_check_pt0 = cv.pointPolygonTest(cnt, (int(pt0[0]), int(pt0[1])), False)
    inside_check_pt1 = cv.pointPolygonTest(cnt, (int(pt1[0]), int(pt1[1])), False)

    while inside_check_pt0 + inside_check_pt1 != -2: # While both points are inside cnt
        if inside_check_pt0 >= 0:
            pt0 = [pt0[0]-(5*vx), pt0[1]-(5*vy)]
        if inside_check_pt1 >= 0:
            pt1 = [pt1[0]+(5*vx), pt1[1]+(5*vy)]
        inside_check_pt0 = cv.pointPolygonTest(cnt, (int(pt0[0]), int(pt0[1])), False)
        inside_check_pt1 = cv.pointPolygonTest(cnt, (int(pt1[0]), int(pt1[1])), False)   

    cv.circle(img, (int(pt0[0]), int(pt0[1])), 3, (0,0,255),2)
    cv.circle(img, (int(pt1[0]), int(pt1[1])), 3, (0,255,0),2)
    
    
    lefty = int((-x*vy/vx) + y)
    righty = int(((cols-x)*vy/vx)+y)
    cv.line(img,(cols-1,righty),(0,lefty),(0,255,0),2)
    cv.imshow('lala', img)
    key = cv.waitKey(0)
    cv.destroyAllWindows() 




"""
# define pts from the question
pts = np.array([[2,3],[4,5],[6,2],[8,8],[10,3],[12,1]])

tck, u = splprep(pts.T, u=None, s=0.0) 
u_new = np.linspace(u.min(), u.max(), 100)
x_new, y_new = splev(u_new, tck, der=0)

plt.plot(pts[:,0], pts[:,1], 'ro', label='Control Points')
plt.plot(x_new, y_new, 'b--', label='B-Spline Curve')
plt.legend(loc=2)
plt.show()
"""
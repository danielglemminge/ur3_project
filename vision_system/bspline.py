

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
    print(cv.pointPolygonTest(cnt, (int(x),int(y)), False))

    cv.putText(img, str(vx) + str(vy)+str(x)+str(y), (50,50), cv.FONT_HERSHEY_SIMPLEX, 1,(255,0,0),2,cv.LINE_AA)
    lefty = int((-x*vy/vx) + y)
    righty = int(((cols-x)*vy/vx)+y)
    cv.line(img,(cols-1,righty),(0,lefty),(0,255,0),2)

    cv.imshow('Img-line', img)

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
import numpy as np
import cv2 as cv
 

path_to_input = "/home/daniel/Master's Thesis/images/Method/"
img = path_to_input+"melanin_mask_com_demo.jpg"



im = cv.imread(img)
assert im is not None, "file could not be read, check with os.path.exists()"
imgray = cv.cvtColor(im, cv.COLOR_BGR2GRAY)
ret, thresh = cv.threshold(imgray, 127, 255, 0)
contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
hull_list = []
for i in contours:
    hull = cv.convexHull(i)
    hull_list.append(hull)

# cv.drawContours(im, contours, -1, (0,255,0), 3)
cv.drawContours(im, hull_list, -1, (0,255,255), 3)

cnt = contours[0]
M = cv.moments(hull_list[0])
cx = int(M['m10']/M['m00'])
cy = int(M['m01']/M['m00'])


cv.circle(im,(cx,cy),2, (0,0,2555),10)
cv.putText(im, '(X,Y)', (cx+10,cy-10), cv.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)


cv.imshow('Contour', im )
cv.waitKey(0)
cv.destroyAllWindows()


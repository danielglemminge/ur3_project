import numpy as np
import cv2
import random
import copy

def p4(p1, p2, p3):
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3
        dx, dy = x2-x1, y2-y1
        det = dx*dx + dy*dy
        a = (dy*(y3-y1)+dx*(x3-x1))/det
        return [x1+a*dx, y1+a*dy]

start = [1,1]
stop = [3,1]
curr = [2,2]

closest = p4(start, stop, curr)
print(closest)
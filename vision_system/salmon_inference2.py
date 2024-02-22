
import cv2
import numpy as np
import os
import copy
import random
# from vision_system.old_ideas.APF import PotentialFieldPlanner
# from vision_system.old_ideas.APF2 import PotentialFieldPlanner2
from APF4 import PotentialFieldPlanner4
from itertools import zip_longest

"""
Starting variables
"""
path_to_input = '/home/daniel/catkin_ws/src/ur3_project/vision_system/'

# def imshow_many(image_list, dimension=(2,1)):
#     if dimension == (2,1):
#         stack_x = np.concatenate((image_list[0], image_list[1]), axis=1)
#     elif dimension = (2,2):
#         stack_y



def limit_pt(pt, width, height):
    # pt = [x,y]
    pt[0] = min(max(0,pt[0]), width)
    pt[1] = min(max(0,pt[1]), height)
    return pt

#########################################################################



def get_scan_start_stop(im, binary_mask):
    img = copy.deepcopy(im)
    img_draw = copy.deepcopy(im)

    height,width = binary_mask.shape[:2]
    
    
    #horizontal = np.concatenate((image, image_draw,binary_mask), axis=1)
    # cv2.imshow('bonary_mask',binary_mask)
  

    # find contour
    contours, _ = cv2.findContours(binary_mask,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    print('Number of contours: ', len(contours))

    # Join the contours if more than 1 are found.
    if (1<len(contours)):
        centroid_list = []
        for index, cnt in enumerate(contours):
            # Momemts_list.append(cv2.moments(cnt))
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
            else:
                cx, cy = 0, 0
            centroid_list.append([cx,cy])
       
        factor = 1
        temp = np.zeros((height,width,3),dtype=np.uint8)
        temp[:,:,0] = binary_mask[:,:]
        temp[:,:,1] = binary_mask[:,:]
        temp[:,:,2] = binary_mask[:,:]
        # print("centroid_list = ", centroid_list)

        # Draw a white line between multiple centroids to joint all of them.
        for index in range(len(centroid_list)-1):
            # print("centroid_list[index] ", centroid_list[index])
            # print("centroid_list[index+1] ", centroid_list[index+1])
            cv2.line(temp,(centroid_list[index]),(centroid_list[index+1]),(255,255,255),10)
            # cv2.line(temp,(0,0),(centroid_list[index]),(255,255,255),1)       # uncomment to generate images for documentation
            # cv2.line(temp,(0,0),(centroid_list[index+1]),(255,255,255),1)     # uncomment to generate images for documentation

        # cv2.imshow("joined contours", cv2.resize(temp,(int(width/factor),int(height/factor)), cv2.INTER_LINEAR))      # uncomment to generate images for documentation
        # cv2.imwrite(path_results + "contours_" + filename, cv2.resize(temp,(int(width/factor),int(height/factor)), cv2.INTER_LINEAR))     # uncomment to generate images for documentation


    rect = cv2.minAreaRect(contours[0])
    box = cv2.boxPoints(rect)                    
    box = np.int0(box)
    # print("Box coordinates ([[x1,y1]...[x4,y4]]) = ", box)
    # cv2.drawContours(img_draw,[box],0,(0,0,255),2)        # uncomment to generate images for documentation
    # cv2.imshow("bbox", img_draw)                          # uncomment to generate images for documentation
    # cv2.imwrite(path_results + "bbox_" + filename, img_draw)      # uncomment to generate images for documentation


    # Cross: create a cross inside the box.
    # Average of boxes' coordinates and then converting to int
    cross = []
    cross.append( [int(i) for i in  ((box[0]+box[1])/2.0)] )
    cross.append( [int(i) for i in  ((box[1]+box[2])/2.0)] )
    cross.append( [int(i) for i in  ((box[2]+box[3])/2.0)] )
    cross.append( [int(i) for i in  ((box[3]+box[0])/2.0)] )
    # Draw crosses on the image
    # cv2.line(img_draw,cross[0], cross[2],(255,255,0),3)       # uncomment to generate images for documentation
    # cv2.line(img_draw,cross[1], cross[3],(0,255,255),3)       # uncomment to generate images for documentation
    # cv2.imshow("cross", img_draw)       # uncomment to generate images for documentation
    # cv2.imwrite(path_results + "cross_" + filename, img_draw)       # uncomment to generate images for documentation

    # Generate a belly line by comparing the l2 norm of the cross's height and width.
    d1 = np.linalg.norm([cross[0], cross[2]])
    d2 = np.linalg.norm([cross[1], cross[3]])
    belly_line = [cross[0], cross[2]] if (d1 >= d2) else [cross[1], cross[3]]    

    # Note order of content ! belly_line = [[x1,y1],[x2,y2]]
    # print("Belly_line[[x1,y1],[x2,y2]] = ", belly_line)    
    # angle = np.arctan2(y,x), get x and y coordinates.    
    x = belly_line[0][0] - belly_line[1][0]
    y = belly_line[0][1] - belly_line[1][1]
    theta = np.arctan2(y,x)
    #print("Theta(rads) = ", theta," Theta(deg) = ", theta * 180/np.pi)

    # theta = abs(theta)

    # Use the length of the belly_line to determine the offset
    belly_norm = np.linalg.norm(belly_line)
    # print("belly_line length = ", belly_norm)
    offset = belly_norm/30

    # Create two points that are +90 and -90 to the belly line. 
    x1,y1 = int(offset * np.cos(theta - np.pi/2)), int(offset * np.sin(theta - np.pi/2))
    x2,y2 = int(offset * np.cos(theta + np.pi/2)), int(offset * np.sin(theta + np.pi/2))

    sum1 = 0
    no_of_pts = 100
    line_1 = np.array([ belly_line[0] + np.array([x1,y1]), belly_line[1] + np.array([x1,y1]) ])
    line_2 = np.array([ belly_line[0] + np.array([x2,y2]), belly_line[1] + np.array([x2,y2]) ])
    # Draw the offset lines.
    # cv2.line(img_draw,line_1[0], line_1[1],(127,127,0),3)       # uncomment to generate images for documentation
    # cv2.line(img_draw,line_2[0], line_2[1],(0,127,127),3)       # uncomment to generate images for documentation
    # cv2.imshow("offset lines", img_draw)       # uncomment to generate images for documentation
    # cv2.imwrite(path_results + "offset_lines_" + filename, img_draw)       # uncomment to generate images for documentation

    line_1_inter = np.linspace(line_1[0], line_1[1], no_of_pts)
    line_2_inter = np.linspace(line_2[0], line_2[1], no_of_pts)   
    
    # Cal sum of intensity along line_1 only on R channel of input image.
    # Note: cv2.line uses the point as [x,y], eg: cv2.line(im,[x1,y1],[x2,y2],color,thickness)
    # However the image coordinates are flipped ! ie: img[y][x]
    for p in line_1_inter:
        # points in the interpolated array are in the form [[x1,y1]...[xn,yn]]
        # [x,y] need to thresholed to [1920-1,1080-1] else the index will exceed bounds.
        x = min(int(p[0]), width-1)
        y = min(int(p[1]), height-1)
        # x = int(p[0])
        # y = int(p[1])
        sum1 = sum1 + img[y,x,2]    # sum = sum + values in Red Channel of img
    # print("LINE_1 : sum1 = ", sum1)

    sum2 = 0
    # Cal sum of intensity along line_2 
    for p in line_2_inter:
        # points in the interpolated array are [[x1,y1]...[xn,yn]]
        # [x,y] need to thresholed to [1920-1,1080-1] else the index will exceed bounds.
        x = min(int(p[0]), width-1)
        y = min(int(p[1]), height-1)
        # x = int(p[0])
        # y = int(p[1])
        sum2 = sum2 + img[y,x,2]    # sum = sum + values in Red Channel of img
    # print("LINE_2 : sum2 = ", sum2)

    # # Decide which line to choose as scan line and return 
    # temp = line_1 if(sum1 >= sum2) else line_2   
    scan_line = line_1 if(sum1 >= sum2) else line_2



    # Decide in image space to stop the points from flipping around.
    # we find max in X between pt0 and pt1, then return the points accordingly, not need to swap.
    pt0 = np.array([ scan_line[0][0], scan_line[0][1] ])      # [x,y]
    pt1 = np.array([ scan_line[1][0], scan_line[1][1] ])      # [x,y]  
    # print("pt0", pt0[0], pt0[1])
    # print("pt1", pt1[0], pt1[1])

    # Sometimes the calculated scan and belly lines will have -ve coordinates because we use trig functions.
    # We need to threshold these to the image resolution. 
    # Opencv automatiically floors or thresholds -ves to 0 and values exceeding the image size are limited to max width or height.
    pt0 = limit_pt(pt0, width, height)
    pt1 = limit_pt(pt1, width, height)



    # A check to stop the flipping of points        
    if ( pt0[0] < pt1[0] ):
        cv2.circle(img, (pt0[0],pt0[1]), 2, (0,255,0),5)
        cv2.circle(img, (pt1[0],pt1[1]), 2, (0,0,255),5)
        return img, pt0, pt1        
    elif ( pt1[0] < pt0[0] ):
        cv2.circle(img, (pt1[0],pt1[1]), 1, (0,255,0),5)
        cv2.circle(img, (pt0[0],pt0[1]), 1, (0,0,255),5)
        return img, pt1, pt0        
    else:
        print("In get_scan_start_stop: Invalid scan points")
        return img, [None,None], [None,None]
    
#########################################################################


def get_black_spot_coord(im, black_spot_mask):
    img_draw = copy.deepcopy(im)

    #black_spot_mask = cv2.cvtColor(black_spot_mask, cv2.COLOR_BGR2GRAY) # grayscale
    ret, thresh = cv2.threshold(black_spot_mask,120,255,cv2.THRESH_BINARY) #binary transformation
    # Find contour
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    #print('Number of spots: ', len(contours))
    hull_list = [cv2.convexHull(contour) for contour in contours]

    centroid_list = []  
    for h in hull_list:
        # calculate moments for each contour
        M = cv2.moments(h)
   
        # calculate x,y coordinate of center
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        centroid_list.append([cX, cY])
    
    return img_draw, hull_list, centroid_list
#########################################################################

def get_binary_masks(image):
    print('Trying to get binary mask')
    belly_mask = cv2.imread(path_to_input+'images_michael/binary_mask_f4.jpg') # Read
    melanin_mask = cv2.imread(path_to_input+'images_michael/melanin_mask_f4.jpg') # Read

    belly_mask = cv2.cvtColor(belly_mask, cv2.COLOR_BGR2GRAY) # Grayscale
    melanin_mask = cv2.cvtColor(melanin_mask, cv2.COLOR_BGR2GRAY) # Grayscale

    ret,belly_mask = cv2.threshold(belly_mask, 120,255,cv2.THRESH_BINARY) # Binary transformation
    ret,melanin_mask = cv2.threshold(melanin_mask, 120,255,cv2.THRESH_BINARY) # Binary transformation

    return belly_mask, melanin_mask



#########################################################################

def inference(input_source='image'):

    if input_source == 'image':
        # Read the image input for further processing
        raw_image = cv2.imread(path_to_input + 'images_michael/input_f4.jpg')
        raw_image = cv2.resize(raw_image, (0, 0), fx = 0.5, fy = 0.5)
        im_draw = copy.deepcopy(raw_image)

        belly_mask, melanin_mask = get_binary_masks(raw_image) # Extracts binary mask from input

        if np.any(belly_mask) != None:

            im_start_stop, pt0, pt1 = get_scan_start_stop(raw_image, belly_mask)
            print('p0, p1:',pt0, pt1)
            

            if np.any(melanin_mask) != None:
                im_black_spot, hull_list, mass_center_list = get_black_spot_coord(raw_image, melanin_mask)



                cv2.circle(im_black_spot, (int(pt0[0]), int(pt0[1])), 0, (0,0,255),5)
                cv2.circle(im_black_spot, (int(pt1[0]), int(pt1[1])), 0, (0,255,0),5)
                cv2.drawContours(im_black_spot, hull_list, -1, (0,0,255), 1)
                #cv2.imshow('im_start_stop', im_start_stop)
                
                planner = PotentialFieldPlanner4(pt0, pt1, hull_list, mass_center_list)

                path = planner.plan()

                print('path computed')

                
        
                for coord in path:
                    cv2.circle(im_black_spot, (int(coord[0]), int(coord[1])), 0, (0,255,0),2)
                                
                cv2.imshow('im_black_spot', im_black_spot)
                #cv2.imwrite('/home/daniel/catkin_ws/src/ur3_project/documentation_images/scan_line/APF4_fish_4.jpg', im_black_spot)


                
                while True:
                    key = cv2.waitKey(1)
                    if key == 27:
                        break
            else:
                print('No black spots found')

            #run calculate z value from (x,y) for the return argument from get_scan_start_stop()
            #Create ros message from it?
        else:
            print('No instance of belly was found')

    elif input_source == 'realsense':
        # Needs to take z-coordinate into account
        print('not yet defined')

    elif input_source == 'video':
        print('not yet defined')
    else:
        print('invalid input_source, choose image, realsense or video')



inference()



"""
To Do:
- Add option for video and realsense feed
- Test repulsive force in only y direction.
- Check out bezier curves
- Negative repulsive force once the fish is past the obstacle 
- Check out magnet north and south pole approach (two circles)

Notes:
- If image should be flipped, remember to flip raw_image and both masks in the separate functions!
- APF has no centerline allignment, while APF2 has one,


"""

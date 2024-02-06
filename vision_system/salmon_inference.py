
import cv2
import numpy as np
import os
import copy
import random
from APF import PotentialFieldPlanner
from APF2 import PotentialFieldPlanner2

"""
Starting variables
"""
path_to_input = '/home/daniel/catkin_ws/src/ur3_project/vision_system/input_images/'

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

def get_binary_mask(image):
    
    """
    global path, filename, path_test, path_results
    print("Trying to get binary mask...")
    outputs = predictor(im)         

    if (0 in outputs['instances'].pred_classes.tolist()):
        # Get the mask from output and convert to a mat array such that its compatible with opencv.            
        mask_array = outputs['instances'].pred_masks.to('cpu').numpy() 
        num_instances = mask_array.shape[0]
        mask_array = np.moveaxis(mask_array, 0, -1)
        mask_array_instance = []
        binary_mask = np.zeros_like(im) # Create a black image
        for i in range(num_instances):
            mask_array_instance.append(mask_array[:, :, i:(i+1)])   
            binary_mask = np.where(mask_array_instance[i] == True, 255, binary_mask)    # Convert only those pixels that are 1 to 255.        
        # cv2.imshow("binary_mask", binary_mask)       # uncomment to generate images for documentation
        # cv2.imwrite(path_results + "binary_mask_" + filename, binary_mask)       # uncomment to generate images for documentation
        return binary_mask        
    else:
        print("No instances of belly.")
        return None
    """
    
    print('Trying to get binary mask')
    binary_mask = cv2.imread(path_to_input+'binary_mask.png')
    #--------------------------------------------------------------------------------------------------------
    # binary_mask = cv2.rotate(binary_mask, cv2.ROTATE_180) #Flip! Must flip raw_image and black spot mask too!
    #--------------------------------------------------------------------------------------------------------
    binary_mask = cv2.cvtColor(binary_mask, cv2.COLOR_BGR2GRAY) # Binary transformation

    return binary_mask


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
    cv2.drawContours(img,[box],0,(0,0,255),2)        # uncomment to generate images for documentation
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


def get_black_spot_mask(image):
    """
    Placeholder for machine learning to extract binary mask for black spot
    Can also alter get_binary_mask() to extract more classes
    """
    black_spot_mask = cv2.imread(path_to_input+'1st_melanin_spot.png')
    #--------------------------------------------------------------------------------------------------------
    # black_spot_mask=cv2.rotate(black_spot_mask, cv2.ROTATE_180) #Flip! Must flip raw_image and other mask too!
    #--------------------------------------------------------------------------------------------------------
    black_spot_mask = cv2.cvtColor(black_spot_mask, cv2.COLOR_BGR2GRAY) # Binary transformation

    return black_spot_mask

def get_black_spot_coord(im, black_spot_mask):
    img_draw = copy.deepcopy(im)
    height,width = black_spot_mask.shape[:2]
  
    # Find contour
    contours, _ = cv2.findContours(black_spot_mask,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    print('Number of spots: ', len(contours))
    black_spot_list = []
    for contour in contours:
        (x,y),radius = cv2.minEnclosingCircle(contour)
        x_y_radius = np.array([int(x), int(y), int(radius)])
        black_spot_list.append(x_y_radius)
        color = (random.randint(0,256), random.randint(0,256), random.randint(0,256))
        cv2.circle(img_draw, x_y_radius[:2], x_y_radius[2], color, thickness=2)
    
    return img_draw, black_spot_list


#########################################################################

def inference(input_source='image'):
    # Top level function

    if input_source == 'image':
        # Read the image input for further processing
        raw_image = cv2.imread(path_to_input + 'salmon_conveyor.png')
        #--------------------------------------------------------------------------------------------------------
        #Flip! Must flip belly mask and black spot mask too!
        # raw_image = cv2.rotate(raw_image, cv2.ROTATE_180)
        #--------------------------------------------------------------------------------------------------------

        binary_mask = get_binary_mask(raw_image) # Extracts binary mask from input
        black_spot_mask = get_black_spot_mask(raw_image)

        if np.any(binary_mask) != None:

            im_start_stop, pt0, pt1 = get_scan_start_stop(raw_image, binary_mask)


            if np.any(get_black_spot_mask) != None:
                im_black_spot, black_spot_list = get_black_spot_coord(raw_image, black_spot_mask)

                # cv2.imshow("image-start-stop", im_start_stop) # Display one pic at the time
                # cv2.imshow("image-black-spot", im_black_spot) # Display one pic at the time

                artificial_obstacle=[np.array([380,280, 20])] # The testing images does not have a path that interfere with the black spot from the binary mask
                cv2.circle(im_black_spot, (artificial_obstacle[0][:2]), artificial_obstacle[0][2], (0,0,255),2)
                planner = PotentialFieldPlanner2(start=pt0, goal=pt1, obstacles=artificial_obstacle)

                # planner = PotentialFieldPlanner(start=pt0, goal=pt1, obstacles=black_spot_list)
                path = planner.plan()
                
                for coord in path:
                    cv2.circle(im_black_spot, (int(coord[0]), int(coord[1])), 2, (0,255,0),1)

                
                horizontal = np.concatenate((im_black_spot, im_start_stop), axis=1)
                cv2.imshow('start stop, and black spot', horizontal)

                
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
To Do/Notes:
- Resize images, maybe after path calculations for higher precision?
- Add option for video and realsense feed
- Test repulsive force in only y direction.

Notes:
- If image should be flipped, remember to flip raw_image and both masks in the separate functions!
- APF has no centerline allignment, while APF2 has one,


"""

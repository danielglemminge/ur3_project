
import cv2
import numpy as np
import os
import copy

"""
Starting variables
"""
path_to_input = '/home/daniel/catkin_ws/src/ur3_project/vision_system/input_images/'

# def imshow_many(image_list, dimension=(2,1)):
#     if dimension == (2,1):
#         stack_x = np.concatenate((image_list[0], image_list[1]), axis=1)
#     elif dimension = (2,2):
#         stack_y




def get_scan_path_2d(image, binary_mask):
    image_draw = copy.deepcopy(image)
    horizontal = np.concatenate((image, image_draw,binary_mask), axis=1)
    cv2.imshow('hori',horizontal)
    cv2.waitKey(0)

    # find contour

    #create bounding box

    #check dimensions of box, and decide long/short side

    #end up with start- and end point, then call calculate_path() from test.py

    # return scan path

    return None

     


        

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
    return binary_mask





def inference(input_source='image'):
    # Top level function

    if input_source == 'image':
        # Read the image input for further processing
        raw_image = cv2.imread(path_to_input + 'salmon_conveyor.png')
        # Resizing?
        #
        #
        binary_mask = get_binary_mask(raw_image) # Extracts binary mask from input
        if np.any(binary_mask) != None:
            get_scan_path_2d(raw_image, binary_mask)

            #run calculate z value from (x,y) for the return argument from get_scan_path_2d()
            #Create ros message from it?
        else:
            print('Noisntance of belly was found')

    elif input_source == 'realsense':
        # Needs to take z-coordinate into aacount
        print('not yet defined')

    elif input_source == 'video':
        print('not yet defined')
    else:
        print('invalid input_source, choose image, realsense or video')


inference()


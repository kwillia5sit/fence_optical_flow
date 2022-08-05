#!/usr/bin/env python3

from colorsys import rgb_to_hls
import numpy as np
import matplotlib.pyplot as plt
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import random
from statistics import mean

#define a global counter variable and set as 0. 
i = 0

# Parameters for lucas kanade optical flow
lk_params = dict(winSize = (9, 9),              #window size each pyramid level
	    		 maxLevel = 4,                       #number of pyramid levels 
		    	 criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,   #termination criteria: Stop after 10 iterations, 
                                                                                #criteria count matches the quality level
			    			4, 0.025),
                 flags = 0,                 #Flags determine what the error is calculating.
                 minEigThreshold = 1e-3)     #Threshold for smallest eignevalue minimum that can ount as good data   
                 #zero/not set flags is error from initial position at prevpts
detector = cv2.AKAZE_create()

#If L1 is less than L1_min don't use the point
L1_min = 3
#If L1 is greater than L1_max don't use the point
L1_max = 500


def add_to_dictionary(good_cur, good_prev):
    #make a dictionary of good points to track that updates each frame (so we can drop points that left)
    track_points_dict = {}
    # Make a loop to put points into an array
    for s, (cur, prev) in enumerate(zip(good_cur, 
                                       good_prev)):
        #Prepare array to be tuples for the line function
        a, b = cur.ravel()
        c, d = prev.ravel()

        #Print the error
        #print("The error for ", cur, "is:", err[s])
        #"L1 distance between new patch and original patch / pixels in window is error"
        L1 = err[s]*9
        #print(err[s]*9) 
        #Note- L1 is some value for optical flow that I don't fully understand
        #error has something to do with the window size and that's why we multiply it by that

        #If error is between the minimum and maximum:
        if L1>L1_min and L1 < L1_max:
            track_points_dict[s] = ('cur', a, b, 
                                    'prev', c, d, 
                                    'color',[random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)])
    return track_points_dict, a, b, c, d

def callback(data):
  #Make the global variables for the named variables
  global prev_frame
  global prev_gray
  global key_points
  global feature_params
  global lk_params
  global i
  global br
  global mask
  global err
  global good_cur
  global good_prev
  global descsi
  global track_points_dict
  #Initialize cv image convertor and AKAZE feature detector
  br = CvBridge()

  #Case 1/2: it's the first frame and we're looking for points to detect
  if (i % 30 == 0):
    # Take first frame of bag message and convert it to openCv image:
    prev_frame = br.imgmsg_to_cv2(data)
    #Take that frame and convert it to a grayscale image:
    prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)
    #Use Gaussian Blur Filter
    prev_gray = cv2.GaussianBlur(prev_gray, (11, 11), 10.0)
    #detect keypoints from the first frame
    key_points, descsi = detector.detectAndCompute(prev_gray, None)
    #Convert key points to coordinate tuples
    key_points = cv2.KeyPoint_convert(key_points)

    #Find the size of the array
    array_size = int(key_points.size)
    my_size = int(array_size/2)
    #Reshape the array to be used by optical flow calculation
    key_points.shape = (my_size, 1, 2)
    # Create a mask image for drawing purposes
    mask = np.zeros_like(prev_frame)
    i = i+1

  else:
    #Take current frame and make it an openCV grayscale
    cur_frame = br.imgmsg_to_cv2(data)
    cur_gray = cv2.cvtColor(cur_frame, cv2.COLOR_BGR2GRAY)
    #Use Gaussian Blur Filter
    cur_gray = cv2.GaussianBlur(cur_gray, (11, 11), 10.0)
    #Show the currnet frame in the "image" window
    cv2.imshow("image", cur_frame)
    cv2.waitKey(1)

	# calculate optical flow
    cur_points, st, err = cv2.calcOpticalFlowPyrLK(prev_gray,
									        cur_gray,
									        key_points, None,
									         **lk_params)
    #Use the resizing array method again on the current points
    cur_size = int(cur_points.size)
    cur_size = int(cur_size/2)
    cur_points.shape = (cur_size, 1, 2)

    #print("initial descs are: ")
    #print(descsi)
    #compute the descriptors for the curret points
    cur_descs = detector.detectAndCompute(cur_gray, None, cur_points)
    #print("current descs are: ")
    #print(cur_descs)

    # Only use good points (had status 1)
    good_cur = cur_points[st == 1]
    good_prev = key_points[st == 1]

    track_dict, a, b, c, d = add_to_dictionary(good_cur, good_prev)
    print(track_dict)

    for tp in range(track_dict):
        #draw a line on the mask
        mask = cv2.line(mask, (int(a), int(b)), (int(c), int(d)), 
                        track_dict[tp][8], 2)
        frame = cv2.circle(cur_frame, (int(a), int(b)), 5,
                        track_dict[tp][8], -1)

    #avg_err= sum(L1)/s
    #print("avg_err = ", avg_err)   
    image = cv2.add(frame, mask)
    cv2.imshow('optical flow', image)
    cv2.waitKey(1)
    key_points = cur_points
    prev_gray = cur_gray
    #add 1 to the counter 
    i = i+1
#end of callback loop

def receive_message():
  # Runs once descs1 nce
  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name. 
  rospy.init_node('optflowLK_node', anonymous=True)
  # Node is subscribing to the sonar oculus node/image topic
  rospy.Subscriber('/sonar_oculus_node/image', Image, callback)
  rospy.spin()
  cv2.destroyAllWindows()

if __name__=='__main__':
  receive_message()
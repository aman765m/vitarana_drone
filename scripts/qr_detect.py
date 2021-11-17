#!/usr/bin/env python
import cv2
import rospy
import numpy as np
from vitarana_drone.msg import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError




img = np.empty([]) # This will contain our image frame from camera
bridge = CvBridge()

def image_callback(data):
    try:
            
        img = bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
            
            
    except CvBridgeError as e:
        print(e)
        return

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    #cam = cv2.VideoCapture(0)

    cv2.namedWindow("test")

    img_counter = 34321

    while True:
        #ret, frame = cam.read()
        '''if not ret:
            print("failed to grab frame")
            break'''
        cv2.imshow("test", gray)
	

        '''k = cv2.waitKey(1)
        if k%256 == 27:
            # ESC pressed
            print("Escape hit, closing...")
            break
        elif k%256 == 32:
            # SPACE pressed
            img_name = "{}.jpg".format(img_counter)
            cv2.imwrite(img_name, gray)
            print("{} written!".format(img_name))
            img_counter += 1'''

            #cam.release()

    cv2.destroyAllWindows()
rospy.Subscriber("/edrone/camera/image_raw", Image, image_callback) #Subscribing to the camera topic


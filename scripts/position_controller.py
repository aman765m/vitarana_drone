#!/usr/bin/env python

'''
# Team ID:          VD_0618
# Theme:            Vitarana Drone
# Author List:      Saurabh Kumar Pandey, Archit Tripathi, Ashwin Kumar, Aman Singh
# Filename:         Position_controller
# Functions:        scheduler(), vel(), gps_callback(), toplaser_callback(), bottomlaser_callback(), image_callback(), marker_detect(), lat_to_x(), long_to_y(), fragmentation(), gripper_check(), landed_on_marker(), landed_on_mat(), path_planner(), pos_control(), pid()      
# Global variables: None
'''

# Importing the required libraries

from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from vitarana_drone.srv import Gripper, GripperResponse, GripperRequest
import pyzbar.pyzbar as pyzbar
from pyzbar.pyzbar import decode
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import rospy
import time
import tf
import math
import csv
from statistics import mean 
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import String
import sys


class Edrone():
    """docstring for Edrone"""
    def __init__(self):
        rospy.init_node('position_controller')  # initializing ros node with name position_control

        self.sample_rate = 50 #sampling for pid

        #order [lat, long, alt]
        self.coordinates = [0.0, 0.0, 0.0] 
        self.set_coordinates = [0.0,0.0,0.0]



        #reading the csv file 
        count_box = 0
        count_ret = 0    

        # results : a temporary variable used for extracting coordinates from csv   
        results = []  

        # dest_array : an array for storing the delivery box coordinates
        self.dest_array = []

        # return_box_array : array for storing the coordinates of return boxes
        self.return_box_array = []

        
        # ret_mat : array for storing coordinates of return mat
        self.ret_mat = [[-7.0, -15.000, 16.447981], [-7.0, -15.000, 16.447981],[-7.0, -15.000, 16.447981], [-7.0, -15.000, 16.447981], [-7.0, -15.000, 16.447981], [-7.0, -15.000, 16.447981], [-7.0, -15.000, 16.447981], [-7.0, -15.000, 16.447981],[-7.0, -15.000, 16.447981]]
        
        # box_coordinates : array for storing coordinates of delivery mat
        self.box_coordinates = [[-21.000000, -15.000000, 16.577979], [-21.000000, -15.000000, 16.577979], [-21.000000, -15.000000, 16.577979],[-21.000000, -15.000000, 16.577979], [-21.000000, -15.000000, 16.577979],[-21.000000, -15.000000, 16.577979],[-21.000000, -15.000000, 16.577979],[-21.000000, -15.000000, 16.577979],[-21.000000, -15.000000, 16.577979]]


        #extracting coordinates from csv
        with open('/home/aman765/catkin_ws/src/vitarana_drone/scripts/bonus.csv') as File:
            reader = csv.reader(File)

            for row in reader:

                if row[0] == 'DELIVERY':
                    sub = list(row[2].strip().split(';'))
                    for i in sub:
                        #print("i_d",i)
                        results.append(float(i))
                    self.dest_array.append(results)
                    results = []

                    #storing the delivery mat coordinates
                    s = row[1]
                    if s[0] == 'A':
                        self.box_coordinates[count_box][1] -= 1.5*(int(s[1])-1)
                    if s[0] == 'B':
                        self.box_coordinates[count_box][0] += 1.5
                        self.box_coordinates[count_box][1] -= 1.5*(int(s[1])-1)
                        #print(box_coordinates,"box")
                    if s[0] == 'C':
                        self.box_coordinates[count_box][0] += 3
                        self.box_coordinates[count_box][1] -= 1.5*(int(s[1])-1)
                    count_box += 1
                else :
                    sub = list(row[1].strip().split(';'))
                    for i in sub:
                        #print("i_r",i)
                        results.append(float(i))
                    self.return_box_array.append(results)
                    results = []

                    #storing the return mat coordinates
                    s = row[2]
                    if s[0] == 'X':
                        self.ret_mat[count_ret][1] -= 1.5*(int(s[1])-1)
                    if s[0] == 'Y':
                        self.ret_mat[count_ret][0] += 1.5
                        self.ret_mat[count_ret][1] -= 1.5*(int(s[1])-1)
                        #print(box_coordinates,"box")
                    if s[0] == 'Z':
                        self.ret_mat[count_ret][0] += 3
                        self.ret_mat[count_ret][1] -= 1.5*(int(s[1])-1)
                    count_ret += 1

        #storing the start position of drone
        self.start_coordinate =  [-12.310000,-23.000000, 16.447981]

        # index : to store the current index to deliver(from scheduler algo) 
        self.index = 0

        # schd_index : to be incremented after each delivery (also to sweep through the array coming from the scheduler algo)
        self.schd_index = 0

        # current_goal : to store the current destination coordinates
        self.current_goal = []
        self.current_goal[:] = self.box_coordinates[self.index]

        # set_coordinates : auxilary variable for the 'current_goal' for feeding it to the pid controller
        self.set_coordinates[:] = self.current_goal[:]


        #storing the building coordinates for further reference
        self.build_height = self.dest_array[0:3][2]

         
        #for the function pos_control(def later)
        self.tim_stamp = time.time()
        
        #k for [lat, long, alt]
        self.Kp = [2.7, 2.7, 185]
        self.Ki = [0.00000, 0.00000,0.02]
        self.Kd = [7.4, 7.4, 170.0]

        # for pid
        #__________________________________
        self.error = [0.0, 0.0, 0.0]
        # d_error : deriviative of error
        self.d_error = [0.0, 0.0, 0.0]
        self.input = [0.0, 0.0, 0.0]
        self.output = [0.0, 0.0, 0.0]
        self.prev_input = [0.0, 0.0, 0.0]
        self.iterm = [0.0, 0.0, 0.0]
        # prev_time = for controlling the frequncy of pid
        self.prev_time = 0.0
        #__________________________________

        # last_time : for time tracking while waiting for the drone to align over the boxes
        self.last_time = 0.0

        # period : for time tracking after deliivery over marker
        self.period = 0.0

        # trigger : check variable for detecting that drone has just landed on marker
        self.trigger = 0

        # now : current time in secs
        self.now = 1000
        
        
        # mono_entry : to enter the condintion only once
        self.mono_entry = 0
        
        # ret_box_attached : to ensure the attachment of box
        self.ret_box_attached = 0
        

        # for spiral ___________________
        self.angle_increment = 0
        self.angle = 0
        self.spiral = False
        #_______________________________

        # attach : command to attach parcel
        self.attach = False

        # range : to store range sensor data
        self.range = [0.0, 0.0, 0.0, 0.0, 0.0]

        # down : "True" means avoid the sensor reading and land
        self.down = False
        self.aligned_on_box = False

        # check_init : to initialize a certain block
        self.check_init = False

        
        #image detection__________________________________________________________________
        # logo_1 : to store the x,y,z,w values of the rectangle during marker detection
        self.logo_1 = [0,0,0,0]
        self.focal_len = 238.350718519 #calculated using the formula given(it is in pixels)

        # camera_check : to open the camera
        self.camera_check = 0

        # array : temporary variable for storing the rectangle data
        self.array = [0, 0,0,0]

        # x, y : distance to be travelled in terms of pixel to reach the marker after detection
        self.x = 0
        self.y = 0

        # x_m, y_m : distance to be travelled in meters to reach the marker after detection
        self.x_m = float("NaN")
        self.y_m = float("NaN")

        # auxilary variables
        self.x_temp = 1000
        self.y_temp = 1000
        #______________________________________________________________________________________

        # threshold : universal thresholding for postioning/alignment
        self.threshold = 0.02

        # schd_array : array containing index values in order of delivery and return (from scheduler_algo)
        self.schd_array = []

        #check : for alternate delivery and return
        self.check = False

        # vel_x, y, z : velocities in m/s from gps
        self.vel_x = 0
        self.vel_y = 0
        self.vel_z = 0

        # lat/long/height_scale : for deciding the magnitude of each unit after fragmentation of the current goal coordinates (for controlling the drone velocity)
        self.lat_scale = 0.5
        self.long_scale = 0.5
        self.height_scale = 0.5

        # variables for publishing_________
        self.pwm_cmd = prop_speed()
        self.pwm_cmd.prop1 = 0.0
        self.pwm_cmd.prop2 = 0.0
        self.pwm_cmd.prop3 = 0.0
        self.pwm_cmd.prop4 = 0.0

        self.data_cmd = edrone_cmd() 
        self.data_cmd.rcRoll = 1500
        self.data_cmd.rcPitch = 1500
        self.data_cmd.rcYaw = 1500
        self.data_cmd.rcThrottle = 1500
        self.data_cmd.aux1 = 0
        self.data_cmd.aux2 = 0
        self.data_cmd.aux3 = 0
        self.data_cmd.aux4 = 0
        
        self.lat_cmd = Float32()
        self.long_cmd = Float32()
        self.alt_cmd = Float32()
        self.lat_cmd.data = 0.0
        self.long_cmd.data = 0.0
        self.alt_cmd.data = 0.0

        

        #for reading image from the camera
        self.image_sub = rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic
        self.img = np.empty([]) # This will contain our image frame from camera
        self.bridge = CvBridge()

        #publishers
        
        self.data_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)


        #subscribers
        rospy.Subscriber('/edrone/gps',NavSatFix , self.gps_callback)
        rospy.Subscriber('/edrone/range_finder_top',LaserScan , self.toplaser_callback)
        rospy.Subscriber('/edrone/range_finder_bottom',LaserScan , self.bottomlaser_callback)
        rospy.Subscriber('/edrone/gps_velocity', Vector3Stamped , self.vel)

        # taking values from the scheduler algo
        rospy.Subscriber('/schedule', String , self.scheduler)


    #callbacks
    def scheduler(self, data):
        '''
        Purpose:
        ---
        To recieve the ordered index list from the scheduler algo and initializing with the coordinates of first parcel

        Input Arguments:
        ---
        data : [string] contains all the indexes of lists containing delivery and return coordinates (in proper order) in conctenated form (from scheduler algo)

        Returns:
        ---
        None

        Example call:
        ---
        *callback function
        '''

        # converting string to int array  
        l = list(data.data.split(','))

        for i in range(len(l)):
            l[i] = int(l[i])
        self.schd_array[:] = l[:]

        # initializing for the first delivery
        if self.check_init == False and self.schd_index < 18:
            self.index = l[self.schd_index]
            self.current_goal[:] = self.box_coordinates[self.index] 
            #self.current_goal = self.return_box_array[self.index][:]
            self.set_coordinates[:] = self.current_goal[:]
            self.current_goal[2]+= 5
            self.check_init = True

    def vel(self, data):

        '''
        Purpose:
        ---
        For getting velocties

        Input Arguments:
        ---
        *standard callback function

        Returns:
        ---
        None

        Example call:
        ---
        *callback function
        '''

        self.vel_x = data.vector.x
        self.vel_y = data.vector.y
        self.vel_z = data.vector.z

        
        return

    def gps_callback(self, msg):

        self.coordinates[0] = msg.latitude
        self.coordinates[1] = msg.longitude
        self.coordinates[2] = msg.altitude


    def toplaser_callback(self, msg):

        self.range[0:4] = msg.ranges[0:4]
        self.range[0] = msg.ranges[4]
        

    def bottomlaser_callback(self, msg):
        self.range[4] = msg.ranges[0] # 5th one is the bottom sensor reading



    def image_callback(self, data):

        '''
        Purpose:
        ---
        Converting the image to OpenCV standard image

        Input Arguments:
        ---
        *standard callback function

        Returns:
        ---
        None

        Example call:
        ---
        *callback function
        '''

        try:
            
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") 
            
        except CvBridgeError as e:
            print(e)
            return

        '''_____________________________________________________________________________________________________________________
        
        #the older bar code detection algo
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        barcodes = pyzbar.decode(self.img) #calling the decode function

        #should decode and store value only when qr code is dected
        if len(barcodes) != 0:
            DATA = barcodes[0].data
            self.li = list(DATA.split(','))
        _______________________________________________________________________________________________________________________'''

    def marker_detect(self,img):

        '''
        Purpose:
        ---
        Marker detection and perimeter approximation

        Input Arguments:
        ---
        img : [array] camera's image after type conversion to cv2 format

        Returns:
        ---
        array : [array] contains the values of x,y,w,h of perimeter

        Example call:
        ---
        demo_array = self.marker_detect(img)
        '''

        logo_cascade = cv2.CascadeClassifier('/home/aman765/backup for vitarana drone/cascade.xml')
        #logo_cascade = cv2.CascadeClassifier('/home/aman765/e_yantra_vd/intro_cascade_classifiers_training_and_usage/data/cascade.xml')

        self.gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
    
        #image, reject levels level weights.
        logo = logo_cascade.detectMultiScale(self.gray, scaleFactor=1.05, minNeighbors=4)
        
        # to ignore unwanted bottom sensor readings
        z = 0
        if self.range[4] > 1:
            z = self.range[4]


        for (x, y, w, h) in logo:
            #print(z*w)
            
            if z*w <= 680 and z*w > 475: #to ensure our poorly trained cascade classifier only store marker values
                
                self.array = [x,y,w,h]
                
                        
                cv2.rectangle(self.img, (x, y), (x + w, y + h), (255, 255, 0), 3)

            
        cv2.imshow("video", self.img)
        cv2.waitKey(5)
        return(self.array)

   

    #for converting lat and long in meters with origin at (19.0, 72.0, 0)
    def lat_to_x(self, input_latitude):
        return 110692.0702932625 * (input_latitude - 19)

    def long_to_y(self, input_longitude):
        return -105292.0089353767 * (input_longitude - 72)

    def fragmentation(self, i, curr_coordinate, scale):
        '''
        Purpose:
        ---
        for dividing the goal into subsets(parts for better speed control)

        Input Arguments:
        ---
        i :  [ int ]
            index to determine whether the given coordinate is latitude, longitude or altitude 

        curr_coordinate :  [float]
            value of the passed coordinate

        Returns:
        ---
        "fragmented coordinate value" if drone is not within 1m range of the dedstination

        Example call:
        ---
        self.fragmentation(0, self.lat_to_x(self.coordinates[0]), 0.25)
        '''

        
        if abs(round(curr_coordinate) - round(self.current_goal[i])) <= 1.0:
            
            return self.current_goal[i]
        else:

            # to give a 'controlled' coordinate in beetween current and destination coordinates to alter the speed
            return curr_coordinate + (self.current_goal[i]-curr_coordinate)/(scale*(abs(round(curr_coordinate) - round(self.current_goal[i]))))

    def gripper_check(self):
        '''
        Purpose:
        
        To call the gripper service
        '''
        
        try:
            
            gripper_activate = rospy.ServiceProxy('/edrone/activate_gripper', Gripper)
            gripper_activate.wait_for_service()
            self.activate = gripper_activate.call(self.attach)
        
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def landed_on_marker(self): 
        '''  
        purpose :
        To check whether it is landed or not on marker
        Input Arguments:
            None
        Returns: 

            Boolean 
        Example call: 
        self.landed_on_marker() 


        '''
        # if self.range[4] > 0.375 and self.range[4] < 0.376:
        #print(self.output[0], self.output[1])
        if (abs(self.x_temp) < 1 and abs(self.y_temp) < 1):
            #print("above the marker")
            # if self.range[4] <0.375 and self.range[4] > 0.371 and abs(self.output[0]) < 0.5 and abs(self.output[1]) < 0.5 :
            #     print(self.range[4])
            #     return True
            # else:
            #     return False
            if self.range[4] > 0.42 and self.range[4] < 0.75 or self.coordinates[2] < self.dest_array[self.index][2]:
                
                return True
            else:
                return False

    def landed_on_mat(self): 
        '''  
        purpose :
        To check whether it is landed or not on mat
        Input Arguments:
            None
        Returns: 

            Boolean 
        Example call: 
        self.landed_on_mat() 
        

        '''

        if self.lat_to_x(self.coordinates[0]) > self.ret_mat[self.index][0]-0.7 and self.lat_to_x(self.coordinates[0]) < self.ret_mat[self.index][0]+0.7 and self.long_to_y(self.coordinates[1]) > self.ret_mat[self.index][1]-0.7 and self.long_to_y(self.coordinates[1]) < self.ret_mat[self.index][1]+0.7:

            if self.range[4] > 0.42 and self.range[4] < 0.75  or self.coordinates[2] < self.ret_mat[self.index][2]+0.5 :
                return True
            else:
                

                return False
        



    #obstacle avoiding algo
    def path_planner(self): 
        '''  
        purpose : 
        To avoid obstacle avoidance and speed controller (controls all path drone is travelling) 
        
        Input Arguments:
            None
        Returns: 

            None
        Example call: 
        self.landed_on_marker() 
        

        '''
        
        #BUG 2 
        

        #for height control 

        if self.range[4] < 1.5:
            # this is used for maintain constant distance from building terrace other than in case of landing 
            if self.down == False:
                self.set_coordinates[2] = self.coordinates[2] + (1.6 - self.range[4])
                
            else:
                
            
                self.set_coordinates[2] = self.fragmentation(2, self.coordinates[2], self.height_scale)

        else:
            
            
            
            self.set_coordinates[2] = self.fragmentation(2, self.coordinates[2], self.height_scale)

        
        # if any obstacle is detected in range of 3m 
        if self.range[3] < 3 and self.down == False:
            
            # until obstacle is cleard move perpendicular to it
            if not self.range[3]>25:
                self.set_coordinates[1] = self.set_coordinates[1] + ((self.current_goal[1] - self.long_to_y(self.coordinates[1]))/abs((self.current_goal[1] - self.long_to_y(self.coordinates[1]))) )*0.5 #keep pitching towards 'neg y' until obstace's edge is reached
            # stop rolling and give 1m back coordinate to counter any drift
            self.set_coordinates[0] = self.lat_to_x(self.coordinates[0]) - (self.lat_to_x(self.coordinates[0])/abs(self.lat_to_x(self.coordinates[0])))#stop roll

            
            
        elif self.range[1] <3 and self.down == False:
            
            
            # until obstacle is cleard move perpendicular to it
            if not self.range[1]>25:
                
                self.set_coordinates[1] = self.set_coordinates[1] + ((self.current_goal[1] - self.long_to_y(self.coordinates[1]))/abs((self.current_goal[1] - self.long_to_y(self.coordinates[1]))) )*0.5 #keep pitching towards 'neg y' until obstace's edge is reached
            # stop rolling and give 1m back coordinate to counter any drift
            self.set_coordinates[0] = self.lat_to_x(self.coordinates[0]) - (self.lat_to_x(self.coordinates[0])/abs(self.lat_to_x(self.coordinates[0]))) #stop roll 
            
            


            
        else :

            # # moving in diagonal
            # if abs(self.lat_to_x(self.coordinates[0]) - self.current_goal[0]) > abs(self.long_to_y(self.coordinates[1]) - self.current_goal[1]) + 2 and abs(self.long_to_y(self.coordinates[1]) - self.current_goal[1]) > 0.001:

            #     self.long_scale *= (abs(self.lat_to_x(self.coordinates[0]) - self.current_goal[0])/abs(self.long_to_y(self.coordinates[1]) - self.current_goal[1]))
            #     print(self.long_scale,"long")

            # elif abs(self.lat_to_x(self.coordinates[0]) - self.current_goal[0]) +2 < abs(self.long_to_y(self.coordinates[1]) - self.current_goal[1]) and abs(self.lat_to_x(self.coordinates[0]) - self.current_goal[0]) > 0.001:

            #     self.lat_scale *= (abs(self.long_to_y(self.coordinates[1]) - self.current_goal[1])/abs(self.lat_to_x(self.coordinates[0]) - self.current_goal[0]))
            #     print(self.lat_scale,"lat")

            # after detaching the parcel on marker move straight for 1.5 second 

            if self.attach == False and self.check == True and self.trigger == 1:
                

                if self.now < self.period + 1.5:
            
                    if abs(self.current_goal[0]) > abs(self.current_goal[1]):
                        self.lat_scale = 100
                    else:
                        self.long_scale = 100
            else:
                self.trigger = 0

            # calling fragmentation function 
            self.set_coordinates[1] =  self.fragmentation(1, self.long_to_y(self.coordinates[1]), self.long_scale)
            self.set_coordinates[0] =  self.fragmentation(0, self.lat_to_x(self.coordinates[0]), self.lat_scale)
                

        #to avoid landing abruptly we are managing the speed of drone by changing height_scale
        if self.range[4] > 0.45 and self.range[4] <1:
            self.height_scale = abs(round(self.vel_z**2))*0.5 + 0.1
            
        else:
            self.height_scale = 0.25
            
            
        # lower lat_scale and long_scale more the speed (speed is iversely proportional to scale)
        #vision of 25 metres - to slow down on seeing the obstacle or reaching destination

        if (self.range[3] < 25 or self.range[1] < 25) or abs(self.current_goal[0]- self.lat_to_x(self.coordinates[0])) < 28:  

            # reducing speed by increasing the lat_scale according to square of cuurent velocity for smooth response
            # we are decreasing speed to avoid any collision or overshoot 
            self.lat_scale = round(((abs(self.vel_x))**2)* 0.1, 2)+ 0.17 
             
            # drone is already well controlled and enough reduce in speed (for velocity less than 2)
            if abs(self.vel_x) < 2:
                if (self.range[3] < 2 or self.range[1] < 2) or abs(self.current_goal[0]- self.lat_to_x(self.coordinates[0])) < 2 :
                    self.lat_scale = round(((abs(self.vel_x)))* 0.5, 2)+ 0.2
                else:

                    self.lat_scale = 0.18
            
        else:
            self.lat_scale = 0.046

        # same procedure as followed for lat

        if (self.range[0] < 25 and self.range[0]>0.3) or (self.range[2] < 25 and self.range[2]>0.3) or abs(self.current_goal[1]- self.long_to_y(self.coordinates[1])) < 28:
            self.long_scale = round(((abs(self.vel_y))**2)* 0.1, 2) + 0.17

            if abs(self.vel_y) < 2:
                if (self.range[0] < 2 and self.range[0]>0.3) or (self.range[2] < 2 and self.range[2]>0.3) or abs(self.current_goal[1]- self.long_to_y(self.coordinates[1])) < 2 :
                    self.long_scale = round(((abs(self.vel_y)))* 0.5, 2) + 0.2
                else:

                    self.long_scale = 0.18
            
        else:
            self.long_scale = 0.046
            

        #side clearance of 3m (here the sensors were giving error values of around 0.25 so the lower limit is 0.3)

        if self.range[0]< 0.18 or (self.range[0]> 0.3 and self.range[0] < 3 ) and self.down == False:
        

            # stop pitching and give 1m back coordinate to counter any drift
            self.set_coordinates[1] = self.long_to_y(self.coordinates[1]) - (self.long_to_y(self.coordinates[1])/abs(self.long_to_y(self.coordinates[1])))#stop pitch 
            # until obstacle is cleard move perpendicular to it

            if not self.range[0]>25:
                
                self.set_coordinates[0] = self.set_coordinates[0] + ((self.current_goal[0] - self.long_to_y(self.coordinates[0]))/abs((self.current_goal[0] - self.long_to_y(self.coordinates[0]))) )*0.5 #keep rolling front until obstace's edge is reached

        
        if self.range[2] < 0.18 or (self.range[2] > 0.3 and self.range[2] < 3) and self.down == False:
        

            # stop pitching and give 1m back coordinate to counter any drift
            self.set_coordinates[1] = self.long_to_y(self.coordinates[1])- 10*(self.long_to_y(self.coordinates[1])/abs(self.long_to_y(self.coordinates[1]))) #stop pitch 

            # until obstacle is cleard move perpendicular to it
            if not self.range[2]>25:
                
                self.set_coordinates[0] = self.set_coordinates[0] + ((self.current_goal[0] - self.long_to_y(self.coordinates[0]))/abs((self.current_goal[0] - self.long_to_y(self.coordinates[0]))) )*0.5  #keep rolling front until obstace's edge is reached
        

                 

        
    def pos_control(self):
        '''  
        Purpose: 
        This is main function and for all functionality such as attaching and detaching and calling support function such as marker_detect or landing function etc we used this function 
        Input Argument:
            None 

        Returns:
            None 
        Example call:
            self.pos_control()
        '''
        # z_m : for calculating drone height by sensor
        z_m = 0 
        # avoiding unwanted values of bottom range sensor
        if self.range[4] > 1:
            z_m = self.range[4]
            
        # coordinate of center of rectangle on marker in image_detection
        centre_x_pixel = self.logo_1[0]+self.logo_1[2]/2       
        centre_y_pixel = self.logo_1[1]+self.logo_1[3]/2
        # calculating the distance(x_error or y_error)
        self.x = 200 - centre_x_pixel
        self.y = 200 - centre_y_pixel 
        self.x_m = -self.x*z_m/self.focal_len
        #0.4 is the zero error
        self.y_m = self.y*z_m/self.focal_len + 0.4 

        # after any image detected 
        if(self.logo_1[2] > 0): 

            # as soon as image detected stop spiralling
            self.spiral = False

            self.x_temp = self.x_m
            self.y_temp = self.y_m

        else:
            #overwriting in unwanted cases
            self.x_m = None 
            self.y_m = None

        # Start detection after attaining some fixed height
        if self.camera_check >0 and self.check == False:
            # to be executed only after drone has reached the given coordinates
            self.logo_1 = self.marker_detect(self.img)  

        # moving towards the detected marker (filtering the unwanted values of x_m and y_m)
        if(self.logo_1[2] > 0) and self.x_m != None and abs(self.x_m)>0.0001 :        
                
            self.current_goal[0:2] = [self.lat_to_x(self.coordinates[0]) + self.x_m, self.long_to_y(self.coordinates[1]) + self.y_m]


        # universal threshold condition
        if self.lat_to_x(self.coordinates[0]) > self.current_goal[0]-self.threshold and self.lat_to_x(self.coordinates[0]) < self.current_goal[0]+self.threshold and self.long_to_y(self.coordinates[1]) > self.current_goal[1]-self.threshold and self.long_to_y(self.coordinates[1]) < self.current_goal[1]+self.threshold:
            
            self.camera_check +=1
            self.down = True
            self.vel_check = False

            # for landing on cases other than marker
            if not (self.attach == True and self.schd_index%2 == 0):
                # delay for alignment
                if self.aligned_on_box == False:
                    time.sleep(0.5)
                    self.aligned_on_box = True

                self.current_goal[2] -=0.1

            # threshold for gripping the parcel

            if ((self.coordinates[2] <= self.box_coordinates[self.index][2] +1 and self.check == False) or (self.coordinates[2] <= self.return_box_array[self.index][2]+1 and self.check == True)) and self.ret_box_attached == 0:
                # camera stop  
                self.camera_check = 0

                self.attach = True
                self.gripper_check()
                if self.activate.result == True:
                    self.ret_box_attached += 1
                # self.check == False for destination and True for return mat coordinate 
                    if self.check == False:
                        # giving destination coordinate after attaching the parcel
                        self.current_goal = [self.lat_to_x(self.dest_array[self.index][0]), self.long_to_y(self.dest_array[self.index][1]), (self.dest_array[self.index][2] +5)]

                    else:
                        # giving return mat after attaching the parcel
                        self.current_goal[:] = self.ret_mat[self.index][:]
                        # for reseting the landing condition
                        self.mono_entry = 0   



                    self.threshold = 0.35
                    # to avoid regular obstacles
                    if self.check == False:

                        self.current_goal[2] = self.dest_array[self.index][2] + 12.5
                    else:
                        self.current_goal[2] = self.ret_mat[self.index][2] + 12.5
            
             
            if(self.logo_1[2] > 0) and self.x_m != None: 

                #keeping in the threshold
                if abs(self.x_m) < 0.35 and abs(self.y_m) < 0.35 and abs(self.x_m) > 0 and abs(self.y_m) > 0:
                    # to avoid abruptly landing by reducing frequency 
                    if self.now >= self.last_time+0.5:
                        self.current_goal[2] -= 1
                        self.last_time = self.now
                    
                    #reseting the variables
                    self.logo_1 = [0.0,0.0,0.0,0.0]
                    self.array = [0.0,0.0,0.0,0.0]
                    self.camera_check = 0
                    self.increment = 0
                    self.mono_entry = 0
                    self.angle = 0
                    

                    self.angle_increment = 0
                    self.prev_rect = 0
                    self.aligned_on_box = False
                    self.last_time = 0.0

            # reached threshold and not detecting the marker so enable the spiral and reduce height 
            else:
                if self.lat_to_x(self.coordinates[0]) > self.lat_to_x(self.dest_array[self.index][0])-self.threshold and self.lat_to_x(self.coordinates[0]) < self.lat_to_x(self.dest_array[self.index][0])+self.threshold and self.long_to_y(self.coordinates[1]) > self.long_to_y(self.dest_array[self.index][1])-self.threshold and self.long_to_y(self.coordinates[1]) < self.long_to_y(self.dest_array[self.index][1])+self.threshold:
                    self.spiral = True
                    #self.current_goal[2] = 21
                    self.current_goal[2] = self.dest_array[self.index][2] + 8

        # calling landed function according to self.check value

        if self.check == True: 

            landed = self.landed_on_mat() 

        else: 

            landed = self.landed_on_marker()

        # main conditon for landing and executing only once that's why we are using mono_entry 
        if landed == True and self.mono_entry == 0: 
            # updating values 
            self.trigger = 1
            self.period = self.now

            self.schd_index +=1 

            self.attach = False
            self.gripper_check()
            
            # updating index value according to schd_array
            if self.schd_index < 18:
                self.index = self.schd_array[self.schd_index] 
            
            
            
            self.ret_box_attached = 0
            self.mono_entry +=1



            
            
            
            
            # updating curren_goal coordinate according to self.check 
            if self.check == False: 
                
                self.check = True
                self.current_goal = [self.lat_to_x(self.return_box_array[self.index][0]), self.long_to_y(self.return_box_array[self.index][1]), (self.return_box_array[self.index][2] +10)]
                


            else:
                

                if self.schd_index >= 18:
                    self.current_goal[:] = self.start_coordinate[:]
                
                else:
                    self.check = False
                    self.current_goal[:] = self.box_coordinates[self.index][:]
                    
                
                    self.current_goal[2]+= 5




                
            self.threshold = 0.05
            # camera stop
            self.camera_check = 0

            
            
            self.down = False  
            
        
        # start spiraling 
        if self.spiral == True:

            #moving in an approximated spiral path
            self.current_goal[0:2] = [self.lat_to_x(self.dest_array[self.index][0]) - self.angle*math.sin(self.angle), self.long_to_y(self.dest_array[self.index][1]) - self.angle*math.cos(self.angle)]
            self.angle += 0.05

            if self.angle >= 3.14 + self.angle_increment and self.angle <= 3.15 + self.angle_increment:
                self.angle_increment = self.angle
                self.current_goal[2] += 1

    def pid(self):
        '''
        Purpose :
            Controlling the drone 
        Input Argument :
            None
        Return :
            None
        Example call:
            self.pid()

        ''' 

        #entry condition from sampling time
        if (self.now - self.prev_time) >= 1/self.sample_rate :
            
            

            
            
            #calculating the errors
            
            self.error[0] = self.set_coordinates[0] - self.lat_to_x(self.coordinates[0])
            self.error[1] = self.set_coordinates[1] - self.long_to_y(self.coordinates[1])
            self.error[2] = self.set_coordinates[2] - self.coordinates[2]
            self.input[0] = self.lat_to_x(self.coordinates[0]) #for d_error
            self.input[1] = self.long_to_y(self.coordinates[1])
            self.input[2] = self.coordinates[2]
            
            
            
            


            #d(error) for diff gain (avoided d(set point) to get rid of deriviative kick)
            for i in range(0,3):
                self.d_error[i] = (self.prev_input[i] - self.input[i])
                
            
            

            #iterm for integral (ki is inside to avoid change of past integral as ki is changed while tuning)
            for i in range(0,3):
                self.iterm[i] += self.Ki[i]*self.error[i]
                
            
            #pid output
            for i in range(0,3):
                self.output[i] = self.Kp[i]*self.error[i] + self.iterm[i] + self.Kd[i]*self.d_error[i]*self.sample_rate
            
            #to hold drone at hovering pwm
            self.pwm_cmd.prop1 = 512.0
            self.pwm_cmd.prop2 = 512.0
            self.pwm_cmd.prop3 = 512.0
            self.pwm_cmd.prop4 = 512.0


            #mapping output for altitude controller
            self.pwm_cmd.prop1 += self.output[2]
            self.pwm_cmd.prop2 += self.output[2]
            self.pwm_cmd.prop3 += self.output[2]
            self.pwm_cmd.prop4 += self.output[2]


            

            #mapping and assigning for attitude
            self.data_cmd.rcRoll = self.output[0]*50 + 1500
            self.data_cmd.rcPitch = -self.output[1]*50 + 1500
            
            

            #max min condt.
            if self.pwm_cmd.prop1 > 1024:
                self.pwm_cmd.prop1 = 1024
            elif self.pwm_cmd.prop1 < 0:
                self.pwm_cmd.prop1 = 0

            if self.pwm_cmd.prop2 > 1024:
                self.pwm_cmd.prop2 = 1024
            elif self.pwm_cmd.prop2 < 0:
                self.pwm_cmd.prop2 = 0

            if self.pwm_cmd.prop3 > 1024:
                self.pwm_cmd.prop3 = 1024
            elif self.pwm_cmd.prop3 < 0:
                self.pwm_cmd.prop3 = 0

            if self.pwm_cmd.prop4 > 1024:
                self.pwm_cmd.prop4 = 1024
            elif self.pwm_cmd.prop4 < 0:
                self.pwm_cmd.prop4 = 0

            #for roll,pitch,yaw
            if self.data_cmd.rcRoll > 2000:
                self.data_cmd.rcRoll = 2000
            elif self.data_cmd.rcRoll < 1000:
                self.data_cmd.rcRoll = 1000

            if self.data_cmd.rcPitch > 2000:
                self.data_cmd.rcPitch = 2000
            elif self.data_cmd.rcPitch < 1000:
                self.data_cmd.rcPitch = 1000

            if self.data_cmd.rcYaw > 2000:
                self.data_cmd.rcYaw = 2000
            elif self.data_cmd.rcYaw < 1000:
                self.data_cmd.rcYaw = 1000

            
            #taking one propeller value as all are same for altd. controller
            self.data_cmd.rcThrottle = self.pwm_cmd.prop1/1.024 + 1000
        
        
            #publish data for attitude node
            self.data_pub.publish(self.data_cmd)

            #updating errrors and time
            for i in range(0,3):
                self.prev_input[i] = self.input[i]
            self.prev_time = self.now

            

if __name__ == '__main__':

    
    e_drone = Edrone()
    time_track = time.time()
    # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    r = rospy.Rate(e_drone.sample_rate)
    while not rospy.is_shutdown():
        e_drone.now = time.time()
        try:
            
            e_drone.path_planner()
            e_drone.pos_control()
            
            e_drone.pid()
            r.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException: pass
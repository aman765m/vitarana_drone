#!/usr/bin/env python

'''import rospy
from vitarana_drone.srv import Gripper, GripperResponse, GripperRequest


class gripper():
	def _init_(self):
		rospy.init_node('node_service_client_gripper')

		self.respo = False


		


	def gripper_check(self):
		rospy.wait_for_service(gripper_service)
		try:
			gripper_status = rospy.ServiceProxy('gripper_service', GripperResponse)
			self.respo = gripper_status()
			
			if self.respo == True:
				gripper_activate = rospy.ServiceProxy('gripper_service', Gripper)
				self.activate = gripper_activate()
				self.activate.activate_gripper == True


		except rospy.ServiceException as e:
			print("Service call failed: %s"%e)


def main():
    eDrone_gripper = gripper()
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            eDrone_gripper.gripper_check()
            r.sleep()
        except rospy.ROSInterruptException:
            rospy.logerr("Shtdown Req")

if __name__ == "__main__":
    main()'''


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

global li 


class Edrone():
    """docstring for Edrone"""
    def __init__(self):
        rospy.init_node('position_controller')  # initializing ros node with name drone_control

        self.sample_rate = 50 #sampling for pid

        #order [lat, long, alt]
        self.coordinates = [0.0, 0.0, 0.0]
        #self.set_coordinates = [0,0,5] #first target coordinate for task2
        self.set_coordinates = [18.993676146, 71.9999999999, 10.65496]
        
        #for the function pos_control(def later)
        self.tim_stamp = time.time()
        
        #k for [lat, long, alt]
        self.Kp = [2.5, 2.5, 185.0]
        self.Ki = [0.000, 0.000,0.02]
        self.Kd = [10.3, 10.3, 170.0]
        
        self.error = [0.0, 0.0, 0.0]
        self.d_error = [0.0, 0.0, 0.0]
        self.prev_input = [0.0, 0.0, 0.0]
        self.input = [0.0, 0.0, 0.0]
        self.prev_time = 0.0
        self.now = 1000
        self.output = [0.0, 0.0, 0.0]
        self.iterm = [0.0, 0.0, 0.0]
        self.max_values = [1024.0, 1024.0, 1024.0, 1024.0]
        self.min_values = [0.0, 0.0 ,0.0 ,0.0]
        self.dists = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.count = 0
        self.z = 0
        self.attach = False
        self.range = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.down = False

        #the ultimate coordinates
        self.goal_coordinates = self.set_coordinates[:]

        #for ensuring gradual descend
        self.increment = 0
        self.increment2 = 0
        
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
        self.li = [0,0,0]

    	#for reading image from the camera
    	self.image_sub = rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic
    	self.img = np.empty([]) # This will contain your image frame from camera
    	self.bridge = CvBridge()

        #publishers
        
        self.data_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)
        self.lat_pub = rospy.Publisher('/latitude_error', Float32, queue_size=1)
        self.long_pub = rospy.Publisher('/longitude_error', Float32, queue_size=1)
        self.alt_pub = rospy.Publisher('/altitude_error', Float32, queue_size=1)


       	#subscribers
        rospy.Subscriber('/edrone/gps',NavSatFix , self.gps_callback)
        rospy.Subscriber('/edrone/range_finder_top',LaserScan , self.toplaser_callback)
        rospy.Subscriber('/edrone/range_finder_bottom',LaserScan , self.bottomlaser_callback)
        #rospy.Subscriber('/pid_tuning_altitude', PidTune, self.altitude_set_pid_callback)
        #rospy.Subscriber('/pid_tuning_roll', PidTune, self.latitude_set_pid_callback)
        #rospy.Subscriber('/pid_tuning_pitch', PidTune, self.longitude_set_pid_callback)


    #callbacks

    def image_callback(self, data):
    	try:
    		self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
    	except CvBridgeError as e:
    		print(e)
    		return
    	barcodes = pyzbar.decode(self.img) #calling the decode function

    	#should decode and store value only when qr code is dected
    	if len(barcodes) != 0:
    		DATA = barcodes[0].data
    		self.li = list(DATA.split(','))

    def gps_callback(self, msg):

        self.coordinates[0] = msg.latitude
        self.coordinates[1] = msg.longitude
        self.coordinates[2] = msg.altitude

    def toplaser_callback(self, msg):

        self.range[0:4] = msg.ranges[0:4]
        

    def bottomlaser_callback(self, msg):
    	self.range[4] = msg.ranges[0] # 5th one is the bottom sensor reading

    '''def waypoint_lat(self):
    	distance = (self.goal_coordinates[0]-self.lat_to_x(self.coordinates[0]))*(self.goal_coordinates[0]-self.lat_to_x(self.coordinates[0])) + (self.goal_coordinates[1]-self.long_to_y(self.coordinates[1]))*(self.goal_coordinates[1]-self.long_to_y(self.coordinates[1]))
    	self.set_coordinates[0] = (self.goal_coordinates[0]+self.lat_to_x(self.coordinates[0]))/math.sqrt(distance)

    def waypoint_long(self):
    	distance = (self.goal_coordinates[0]-self.lat_to_x(self.coordinates[0]))*(self.goal_coordinates[0]-self.lat_to_x(self.coordinates[0])) + (self.goal_coordinates[1]-self.long_to_y(self.coordinates[1]))*(self.goal_coordinates[1]-self.long_to_y(self.coordinates[1]))
    	
    	self.set_coordinates[1] = (self.goal_coordinates[1]+self.long_to_y(self.coordinates[1]))/math.sqrt(distance)'''
    	
    
        
    '''def altitude_set_pid_callback(self, msg):

        self.Kp[2] = msg.Kp * 0.1  
        self.Ki[2] = msg.Ki * 0.008
        self.Kd[2] = msg.Kd * 0.1

    def latitude_set_pid_callback(self, msg):

        self.Kp[0] = msg.Kp * 0.1 
        self.Ki[1] = msg.Ki * 0.0008
        self.Kd[2] = msg.Kd * 100000

    def longitude_set_pid_callback(self, msg):

        self.Kp[0] = msg.Kp * 0.1  
        self.Ki[1] = msg.Ki * 0.0008
        self.Kd[2] = msg.Kd * 0.1'''

    #for converting lat and long in meters with origin at (19.0, 72.0, 0)
    def lat_to_x(self, input_latitude):
    	return 110692.0702932625 * (input_latitude - 19)

    def long_to_y(self, input_longitude):
    	return -105292.0089353767 * (input_longitude - 72)

    def gripper_check(self):
		
		try:
			
			gripper_activate = rospy.ServiceProxy('/edrone/activate_gripper', Gripper)
			gripper_activate.wait_for_service()
			self.activate = gripper_activate.call(self.attach)
		
		except rospy.ServiceException as e:
			print("Service call failed: %s"%e)



    #path planning algo
    def path_planner(self):
    	
    	
    	#BUG 2 in 2d (i.e. height is kept fixed at 25m)
    	
    	self.set_coordinates[2] = self.goal_coordinates[2]


    	if self.range[3] <15:

    		self.set_coordinates[0] = self.lat_to_x(self.coordinates[0])+(25-(self.range[3])) #stop roll
    		if not self.range[3]>25:
    			
    			self.set_coordinates[1] -= 0.2 #keep pitching left until obstace's edge is reached


    		
    	else :
    		
    		#self.waypoint_lat()
    		#self.waypoint_long()
    		
    		self.set_coordinates[0] = self.goal_coordinates[0]
    		self.set_coordinates[1] = self.goal_coordinates[1]
    		

    	#side clearance of 2m (here the sensor reading was unexpectedly scaled down by nearly a factor of hundred)

    	if self.range[0]<2/100:
    		
    		self.set_coordinates[1] = self.long_to_y(self.coordinates[1])+(2-(self.range[0])) #stop pitch
    		if not self.range[0]>25:
    			
    			self.set_coordinates[0] -= 0.1 #keep rolling front until obstace's edge is reached

    	
    	if self.range[2]<2/100:
    		
    		self.set_coordinates[1] = self.long_to_y(self.coordinates[1])+(2-(self.range[2])) #stop pitch
    		if not self.range[2]>25:
    			
    			self.set_coordinates[0] -= 0.1 #keep rolling front until obstace's edge is reached


    #For upadting the coordinates (specially for this task)
    def pos_control(self):
    	

    	#keeping a threshold of 0.1m
    	if self.lat_to_x(self.coordinates[0]) < 19.05 and self.lat_to_x(self.coordinates[0]) > 18.94 and self.long_to_y(self.coordinates[1]) < 72.05 and self.long_to_y(self.coordinates[1]) > 71.95:
    		
    		
    		self.goal_coordinates[2] = 15 - self.increment #assuring steady decrement

    		#to control the decrement rate from self.increment
    		if not self.coordinates[2]< 8.25:
    			self.increment +=0.1
    			
    		self.attach = True
    		self.gripper_check()

    		if self.activate.result  == True:
    			
    			self.goal_coordinates = [self.lat_to_x(float(self.li[0])), self.long_to_y(float(self.li[1])), 15] #update the lat and long acc to qr but keep height at 25m
    			
    	#keeping a threshold of 1m for the end position
    	if self.lat_to_x(self.coordinates[0]) < 0.5 and self.lat_to_x(self.coordinates[0]) > -0.5 and self.long_to_y(self.coordinates[1]) < 0.5 and self.long_to_y(self.coordinates[1]) > -0.5:

    		self.goal_coordinates = [0,0,15-self.increment2]

    		if not self.coordinates[2]< float(self.li[2]):
    			self.increment2 +=0.08
    			
    		#if self.coordinates[2]< float(self.li[2]):
    			
    			#self.attach = False


    		


    def pid(self):

    	#entry condition from sampling time
    	if (self.now - self.prev_time) >= 1/self.sample_rate :
    		
    		

    		
    		
    		#calculating the errors
    		
    		self.error[0] = self.set_coordinates[0] - self.lat_to_x(self.coordinates[0])
    		self.error[1] = self.set_coordinates[1] - self.long_to_y(self.coordinates[1])
    		self.error[2] = self.set_coordinates[2] - self.coordinates[2]
    		self.input[0] = self.lat_to_x(self.coordinates[0]) #for d_error
    		self.input[1] = self.long_to_y(self.coordinates[1])
    		self.input[2] = self.coordinates[2]
    		#if i == 0 or i == 1:
    		#self.error[i] *= 100000 #scaling errors to match the magnitude of other attributes(for latitude and longitude)

			

    		#assigning errors
    		self.lat_cmd.data = self.error[0]
    		self.long_cmd.data = self.error[1]
    		#self.alt_cmd.data = self.error[2]
    		#publishing the errors
    		self.lat_pub.publish(self.lat_cmd)
    		self.long_pub.publish(self.long_cmd)

    		
    		self.alt_cmd.data = self.error[2]
    		self.alt_pub.publish(self.alt_cmd)
    		
    		
    		


    		#d(error) for diff gain (avoided d(set point) to get rid of deriviative kick)
    		for i in range(0,3):
    			self.d_error[i] = (self.prev_input[i] - self.input[i])
    			#if i == 0 or i == 1:
					#self.d_error[i] *= 100000 #scaling d_errors to match the magnitude of other attributes(for latitude and longitude)

    		
    		

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
    r = rospy.Rate(e_drone.sample_rate)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    while not rospy.is_shutdown():
        e_drone.now = time.time()
        try:
        	#e_drone.image_callback()
        	e_drone.pos_control()
        	#e_drone.gripper_check()
        	e_drone.path_planner()
        	e_drone.pid()
        	r.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException: pass

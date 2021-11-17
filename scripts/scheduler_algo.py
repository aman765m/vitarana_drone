#!/usr/bin/env python

'''
# Team ID:          VD_0618
# Theme:            Vitarana Drone
# Author List:      Saurabh Kumar Pandey, Archit Tripathi, Ashwin Kumar, Aman Singh
# Filename:         Scheduler_algo
# Functions:        linear_distance(), lat_to_x(), long_to_y(), parcel_delivery(), parcel_return(), create_csv(), publishing(), calling_functions()   
# Global variables: None
'''

import csv
from vitarana_drone.msg import *
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import String
import sys
from rosgraph_msgs.msg import Clock

class Scheduler():
	def __init__(self): 

	# initializing ros node with name order_control

		rospy.init_node('order_controller')  

		self.schedule_cmd =  String()

		# mono_entry : to enter the condintion only once
		self.mono_entry = 0
		self.mono_entry_2 = 0
		self.mono_entry_3 = 0


		self.schedule_pub = rospy.Publisher('/schedule',  String, queue_size=1)


		#reading the csv fileself.

		# ret_mat : array for storing coordinates of return mat
		self.ret_mat = [[-7.0, -15.000, 16.447981], [-5.5, -15.000, 16.447981],[-4.0, -15.000, 16.447981], [-7.0, -16.5000, 16.447981], [-5.50, -16.5000, 16.447981], [-4.0, -16.5000, 16.447981], [-7.0, -18.000, 16.447981], [-5.50, -18.000, 16.447981],[-4.0, -18.000, 16.447981]]

		# box_coordinates : array for storing coordinates of delivery mat
		self.box_coordinates = [[-21.000000, -15.000000, 16.577979], [-19.500000, -15.00000, 16.577979], [-18.000000, -15.0, 16.577979],[-21.000000, -16.5000000, 16.577979], [-21.000000, -18.000000, 16.577979],[-19.500000, -16.500000, 16.577979],[-19.500000, -18.00000, 16.577979],[-18.000000, -16.50, 16.577979],[-18.000000, -18.0, 16.577979]]

		# distances : for storing the linear distance
		self.distances = [0]*10

		

		# dest_array : an array for storing the delivery box coordinates
		self.dest_array = []

		# dest_array_out : an array for storing the delivery box coordinates without any alteration to store it in output csv
		self.dest_array_out = []

		# return_array : array for storing the coordinates of return boxes
		self.return_array = []

		# return_array_out : array for storing the coordinates of return boxes without any alteration to store it in output csv
		self.return_array_out = []

		# dict_dist/dict_return : dictionary that stores index and corresponding linear distance
		self.dict_dist = {}
		self.dict_return = {}

		# schedule_dest/ret : array containing indexes of corresponding distance in descending order
		self.schedule_dest = []
		self.schedule_ret = []
		# results : a temporary variable used for extracting coordinates from csv  
		results = []
		self.ret_var = []
		# extracting coordinates from csv
		with open('/home/aman765/catkin_ws/src/vitarana_drone/scripts/bonus.csv') as File:
			reader = csv.reader(File)

			for row in reader:

				if row[0] == 'DELIVERY':
					sub = list(row[2].strip().split(';'))
					for i in sub:
						#print("i_d",i)
						results.append(float(i))
					self.dest_array.append(results)
					self.dest_array_out.append(row)
					results = []
				else :
					sub = list(row[1].strip().split(';'))
					for i in sub:
						#print("i_r",i)
						results.append(float(i))
					self.return_array.append(results)
					self.return_array_out.append(row)
					results = []


		


	def linear_distance(self,x1,y1,x2,y2):

		return ((x1-x2)**2 + (y1-y2)**2)**0.5

	def lat_to_x(self,input_latitude):
		return 110692.0702932625 * (input_latitude - 19)

	def long_to_y(self,input_longitude):
		return -105292.0089353767 * (input_longitude - 72)


	def parcel_delivery(self):
		'''
		Purpose:
			to arrange delivery coordinates in descending order of linear distance
		Input arguments:
		dest_array : [array] 
			contains the destination coordinates
		box_coordinates : [array]
			contains the coordinates for parcels on delivery mat
		Return:
			None
		Example call:
			scheduler_algo(dest_array, box_coordinates)
		'''


		for i in range(9):
			self.dict_dist[i] = self.linear_distance(self.lat_to_x(self.dest_array[i][0]),self.long_to_y(self.dest_array[i][1]),self.box_coordinates[i][0],self.box_coordinates[i][1])


		for m in range(9):

			
			tets = max(self.dict_dist, key=self.dict_dist.get)
			self.dict_dist[tets] = 0

			self.schedule_dest.append(tets)


	def parcel_return(self):

		'''
		Purpose:
			to arrange return box coordinates in descending order of linear distance from particular marker

		Input arguments:

		dest_array : [array] 
			contains the destination coordinates

		return_array : [array]
			contains the coordinates for parcels on return mat

		schedule_dest : [array]
			contains the arranged destinaion coordinates for deliver boxes

		Return:
			None

		Example call:
			parcel_ret(dest_array, return_array)
		'''

		# index_ret : initializing local variable
		index_ret = 0
		
		for i in range(9):
			for j in range(9):

				self.dict_return[j] = self.linear_distance(self.lat_to_x(self.dest_array[self.schedule_dest[i]][0]),self.long_to_y(self.dest_array[self.schedule_dest[i]][1]),self.lat_to_x(self.return_array[j][0]),self.long_to_y(self.return_array[j][1]))
			index_ret = min(self.dict_return, key=self.dict_return.get)
			self.return_array[index_ret][0:2] = [sys.maxsize, sys.maxsize]
			self.schedule_ret.append(index_ret)


	# calling the functions 
	def calling_functions(self):

		'''
		Purpose:
			collectively calling few functions
		Input arguments:
			none
		Return:
			None
		Example call:
			self.calling_functions()
		'''
		self.parcel_delivery()
		self.parcel_return()
		# merging both outputs from both the functions (it contains alternate values of delivery and return coordinates)
		for i in range(9):
			self.ret_var.append(self.schedule_dest[i])
			self.ret_var.append(self.schedule_ret[i])


	def publishing(self):

		'''
		Purpose:
			Publishing the ordered indexes on form of srting
		Input arguments:
			none
		Return:
			None
		Example call:
			self.publishing()
		'''
		# concatenating in terms of string
		
		self.schedule_out =','.join(str(v) for v in self.ret_var)

		self.schedule_cmd.data = self.schedule_out
		self.schedule_pub.publish(self.schedule_cmd)
		#print(self.schedule_cmd.data, "ret_var")




	def create_csv(self):

		'''
		Purpose:
			creating csv
		Input arguments:
			none
		Return:
			None
		Example call:
			self.create_csv()
		'''

		filename = "/home/aman765/Downloads/sequenced_manifest_bonus.csv"
		# writing to csv file  
		with open(filename, 'w') as csvfile:  
			# creating a csv writer object  
			csvwriter = csv.writer(csvfile)  
				

			for i in range(9):
				csvwriter.writerow(self.dest_array_out[self.schedule_dest[i]])
				csvwriter.writerow(self.return_array_out[self.schedule_ret[i]])


    	



if __name__ == '__main__':

	scheduler = Scheduler()
	scheduler.calling_functions()

	while not rospy.is_shutdown():
		try:
			scheduler.publishing()
			if scheduler.mono_entry_3 == 0 :
				scheduler.create_csv()
				scheduler.mono_entry_3 +=1
			
		except rospy.exceptions.ROSTimeMovedBackwardsException: pass
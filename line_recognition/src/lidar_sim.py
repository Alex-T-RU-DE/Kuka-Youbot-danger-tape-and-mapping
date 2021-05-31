#!/usr/bin/env python
import numpy as np
import rospy
import time
import tf
import tf2_ros
from tape_coordinate.msg import tape_msgs
from tape_coordinate.msg import tape_msgs_array
import math
import rospy.rostime
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import LaserScan


class image_rec(object):
	
	def __init__(self):


		self.lidar_params=rospy.wait_for_message("/base_scan", LaserScan, timeout=None)
		self.lidar_central_beam_index = int(- self.lidar_params.angle_min / self.lidar_params.angle_increment)
		self.lidar_array = list(self.lidar_params.ranges)
		self.camera_params = rospy.wait_for_message("/camera/aligned_depth_to_color/camera_info", 
												  CameraInfo, 
												  timeout = None)
		self.lidar_params.header.frame_id = "laser"
		self.lidar_pub = rospy.Publisher("/tape_scan", LaserScan, queue_size = 100)		
		self.tfBuffer = tf2_ros.Buffer()
	        self.listener = tf2_ros.TransformListener(self.tfBuffer)
		time.sleep(1)
		self.basetocam = self.tfBuffer.lookup_transform('base_laser_front_link', 'arm_camera_link',  rospy.Time(), rospy.Duration(1.0))	



	def getTF(self):

	    q = [self.basetocam.transform.rotation.x, 
	         self.basetocam.transform.rotation.y, 
	         self.basetocam.transform.rotation.z, 
	         self.basetocam.transform.rotation.w]
	    R = tf.transformations.quaternion_matrix(q)	   
	    T = np.array([
	    self.basetocam.transform.translation.x, 
	    self.basetocam.transform.translation.y, 
	    self.basetocam.transform.translation.z,
	    1
	    ])	    	    
	    groundToCam = np.array([
	    [R[0][0], R[0][1], R[0][2], T[0]],
	    [R[1][0], R[1][1], R[1][2], T[1]], 
	    [R[2][0], R[2][1], R[2][2], T[2]],
	    [0, 0, 0, 1]
	    ])

	    return groundToCam,T

	def getObjXYZ(self, line_coordinate):
	    	    
	    cameraMatrix = np.array([
		[self.camera_params.P[0], self.camera_params.P[1], self.camera_params.P[2],self.camera_params.P[3]],
		[self.camera_params.P[4], self.camera_params.P[5], self.camera_params.P[6],self.camera_params.P[7]],
		[self.camera_params.P[8], self.camera_params.P[9], self.camera_params.P[10],self.camera_params.P[11]],
		[0, 0, 0, 1]
		])	 			   
	    RT, T = self.getTF()
	    pix = np.array([line_coordinate.posX, line_coordinate.posY, 1, 1])
	    invM = np.linalg.inv(cameraMatrix)
	    camFrame = invM.dot(pix)
	    camFrame[3] = 0
	    worldFrame = RT.dot(camFrame)
	    scale = (0 - T[2]) / worldFrame[2]
	    scaledVector = scale * worldFrame
	    pos = scaledVector+T	    
	    rospy.loginfo(pos[:2])

	    return pos
		
	def new_lidar(self):
		for i in range(len(self.lidar_params.ranges)):
			self.lidar_array[i] = float("nan")
		line_data = rospy.wait_for_message("/line_coordinates", tape_msgs_array, timeout=None)	
		#test
		self.lidar_array[0] = 2
		self.lidar_array[len(self.lidar_params.ranges) - 1] = 2
		for i in range(0,len(line_data.tape_msgs_array)):	
				
			world_coordinates = self.getObjXYZ(line_data.tape_msgs_array[i])			
			dist_to_point = math.sqrt(pow(world_coordinates[0], 2) + pow(world_coordinates[1], 2))
			angle = math.atan2(world_coordinates[1], world_coordinates[0])
			self.lidar_array[self.lidar_central_beam_index + int(angle / self.lidar_params.angle_increment)] = math.sin(angle) * dist_to_point + math.cos(angle) * dist_to_point
		self.lidar_params.ranges = tuple(self.lidar_array)
		self.lidar_pub.publish(self.lidar_params)			
		rospy.loginfo("new scans published")
        
def main():
 rospy.init_node('fake_lidar_node', anonymous = True)
 image_rec_obj = image_rec()
 while not rospy.is_shutdown():		
	image_rec_obj.new_lidar()
	rate = rospy.Rate(100)
	try:
		rate.sleep()
	except KeyboardInterrupt:
		print("Shutting down the program")
		

if __name__ == '__main__':
	main()

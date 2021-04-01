#!/usr/bin/env python
import numpy as np
import rospy
import time
import tf
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from tape_coordinate.msg import tape_msgs
from tape_coordinate.msg import tape_msgs_array
import math
import rospy.rostime
#SIZE OF THE MAP 1472x1472 points with resolution 0.02m

class image_rec(object):
	
	def __init__(self):
		#define all the variables
		
		#point of the center of RVIZ 
		self.map_center_point=750
		#gettin the params of the camera
		self.camera_params=rospy.wait_for_message("/camera/aligned_depth_to_color/camera_info", 
												  CameraInfo, 
												  timeout=None)
		#creating empty map
		self.grid=OccupancyGrid()
		self.grid.info.resolution=0.02
		self.grid.info.width=1472
		self.grid.info.height=1472
		self.grid.header.frame_id="new_map"
		self.grid.info.map_load_time=rospy.rostime.Time()
		self.grid.info.origin.position.x=60.0
		self.grid.info.origin.position.y=60.0
		self.grid.info.origin.position.z=0
		self.grid.info.origin.orientation.x=0
		self.grid.info.origin.orientation.y=0
		self.grid.info.origin.orientation.z=0
		self.grid.info.origin.orientation.w=1.0
		self.grid.data=tuple(np.zeros(self.grid.info.width*self.grid.info.height))
		self.map_array=list(self.grid.data)
		
		#defining publishers and subsribers	
		self.map_pub = rospy.Publisher("/new_map", OccupancyGrid, queue_size = 100)	
		self.tfBuffer = tf2_ros.Buffer()
	        self.listener = tf2_ros.TransformListener(self.tfBuffer)	
	    	time.sleep(1)


	def getTF(self, basetocam):

	    q = [basetocam.transform.rotation.x, 
	         basetocam.transform.rotation.y, 
	         basetocam.transform.rotation.z, 
	         basetocam.transform.rotation.w]
	    R = tf.transformations.quaternion_matrix(q)
	    #rospy.loginfo(q)
	   
	    T = np.array([
	    basetocam.transform.translation.x, 
	    basetocam.transform.translation.y, 
	    basetocam.transform.translation.z,
	    1
	    ])	    
	    #rospy.loginfo(T[:3])	    
	    groundToCam = np.array([
	    [R[0][0], R[0][1], R[0][2], T[0]],
	    [R[1][0], R[1][1], R[1][2], T[1]], 
	    [R[2][0], R[2][1], R[2][2], T[2]],
	    [0, 0, 0, 1]
	    ])
	    #rospy.loginfo(groundToCam)
	    return groundToCam,T

	def getObjXYZ(self, line_coordinate, basetocam):
	    	    
	    #cameraMatrix = np.array([
		#[596.83, 0, 317.44, 0],
		#[0, 598.09, 244.35, 0], 
		#[0, 0, 1, 0],
		#[0, 0, 0, 1]
		#])
		
		cameraMatrix=np.append(self.camera_params.P, 
							   np.array([[0.0, 0.0, 0.0, 1]]),
							   axis=0)
	    RT,T = self.getTF(basetocam)
	    pix = np.array([line_coordinate.posX, line_coordinate.posY, 1, 1])
	    invM = np.linalg.inv(cameraMatrix)
	    camFrame = invM.dot(pix)
	    camFrame[3] = 0
	    worldFrame = RT.dot(camFrame)
	    scale =(0-T[2])/worldFrame[2]
	    scaledVector = scale*worldFrame
	    pos = scaledVector+T	    
	    #rospy.loginfo(pos[:2])
	    return pos
		
	def new_map(self):
		
		#collecting latest data about line and position of the robot on the map
		line_data=rospy.wait_for_message("/line_coordinates", tape_msgs_array, timeout=None)
		basetocam = self.tfBuffer.lookup_transform('odom', 
												   'arm_camera_link', 
												   rospy.Time())
		#print(len(line_data.tape_msgs_array))
		
		#computing position of each line's square in map
		for i in range(0,len(line_data.tape_msgs_array)):	
				
			world_coordinates=self.getObjXYZ(line_data.tape_msgs_array[i])			
			#in order to understand where does these poits we counting in a such a way
			#that can be better understanded in X and Y coordinates					
			x=(self.map_center_point+int(world_coordinates[1]/self.grid.info.resolution))
			y=(self.map_center_point+int((world_coordinates[0])/self.grid.info.resolution))
			
			#transforming X:Y coonrinates in the number of OccupancyGrid.data[] array format
			#and assign value "occupied" (100) to this position
			self.map_array[(x*self.grid.info.height+y)]=100							
		self.grid.data=tuple(self.map_array)
		self.map_pub.publish(self.grid)			
		rospy.loginfo("New points added to the map")
        
def main():
 rospy.init_node('remapping_node2', anonymous=True)
 image_rec_obj=image_rec()
 while not rospy.is_shutdown():		
	image_rec_obj.new_map()
	rate=rospy.Rate(100)
	try:
		rate.sleep()
	except KeyboardInterrupt:
		print("Shutting down the program")
		

if __name__=='__main__':
	main()

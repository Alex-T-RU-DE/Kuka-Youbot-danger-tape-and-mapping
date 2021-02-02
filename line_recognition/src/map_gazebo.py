#!/usr/bin/env python
import numpy as np
import rospy
import time
import tf
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from tape_coordinate.msg import tape_msgs
from tape_coordinate.msg import tape_msgs_array
import math
import time
import rospy.rostime
#SIZE OF THE MAP 1472x1472 points with resolution 0.02m

class image_rec(object):
	
	def __init__(self):
		#define all the variables and creating empty map
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
		self.grid.data=tuple(np.zeros(1472*1472))
		self.data1=tape_msgs()
		self.lol=list(self.grid.data)
		self.map_pub = rospy.Publisher("/new_map", OccupancyGrid, queue_size = 100)
		self.tfBuffer = tf2_ros.Buffer()
	        self.listener = tf2_ros.TransformListener(self.tfBuffer)	
	    	time.sleep(1)


	def getTF(self):
	    basetocam = self.tfBuffer.lookup_transform('odom', 'camera_link', rospy.Time())
	    #rotaton of last joint(arm_camera_link) relative to first one (base_footprint/odom) 
	    q = [basetocam.transform.rotation.x, 
	         basetocam.transform.rotation.y, 
	         basetocam.transform.rotation.z, 
	         basetocam.transform.rotation.w]
	    R = tf.transformations.quaternion_matrix(q)
	    #rospy.loginfo(q)
	    #distance from last joint(arm_camera_link) relative to first one (base_footprint/odom
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

	def getObjXYZ(self, data):
	    cameraMatrix = np.array([
		[596.83, 0, 317.44, 0],
		[0, 598.09, 244.35, 0], 
		[0, 0, 1, 0],
		[0, 0, 0, 1]
		])
	    
	    RT,T = self.getTF()
	    #data=rospy.wait_for_message("/line_coordinates", tape_msgs, timeout=None)
	    pix = np.array([data.posX, data.posY, 1, 1])
	    #pix = np.array([1135, 279, 1, 1])
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
		
		data1=self.data1
		line_data=rospy.wait_for_message("/line_coordinates", tape_msgs_array, timeout=None)
		#print(len(line_data.tape_msgs_array))
		for i in range(0,len(line_data.tape_msgs_array)):
			data=self.getObjXYZ(line_data.tape_msgs_array[i])					
			x=(750+int(data[1]/0.02))
			y=(750+int((data[0])/0.02))
			self.lol[(x*1472+y)]=100							
		self.grid.data=tuple(self.lol)		
		self.map_pub.publish(self.grid)			
		rospy.loginfo("Updated")
        
        
def main():
 rospy.init_node('remapping_node2', anonymous=True)
 image_rec_obj=image_rec()
 while not rospy.is_shutdown():		
	image_rec_obj.new_map()
	rate=rospy.Rate(100)
	try:
		rate.sleep()
		#rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down the program")
		

if __name__=='__main__':
	main()

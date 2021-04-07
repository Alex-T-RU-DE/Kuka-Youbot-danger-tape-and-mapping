import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
from tape_coordinate.msg import tape_msgs
            
def empty(a):
	pass


class image_rec(object):
	
	FLAG=True
	msgobj=tape_msgs()
  
	
	def __init__(self):

		self.image_sub=rospy.Subscriber("/usb_cam/image_raw", Image, self.camera_callback)
		self.bridge_object=CvBridge()

	
	def camera_callback(self,data):
		try:
			cv_image=self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
			new_cv_image = self.recognition(cv_image)
			#cv2.imshow('frame',  new_cv_image)
		except CvBridgeError as e:
			print("Error")
        
	def recognition(self,image_array):
		
			#ret, image_array = cap.read()
			#image_array=cv2.imread('yellow.1.jpg')
			#image_array_hsv=cv2.cvtColor(image_array,cv2.COLOR_BGR2HSV)			
			cv2.namedWindow("HSV")
			cv2.resizeWindow("HSV",640,240)
			cv2.createTrackbar("HUE min", "HSV", 0, 179, empty)
			cv2.createTrackbar("HUE max", "HSV", 179, 179, empty)
			cv2.createTrackbar("SAT min", "HSV", 0, 255, empty)
			cv2.createTrackbar("SAT max", "HSV", 255, 255, empty)
			cv2.createTrackbar("VALUE min", "HSV", 0, 255, empty)
			cv2.createTrackbar("VALUE max", "HSV", 255, 255, empty)

			h_min=cv2.getTrackbarPos("HUE min", "HSV")
			h_max=cv2.getTrackbarPos("HUE max", "HSV")
			s_min=cv2.getTrackbarPos("SAT min", "HSV")
			s_max=cv2.getTrackbarPos("SAT max", "HSV")
			v_min=cv2.getTrackbarPos("VALUE min", "HSV")
			v_max=cv2.getTrackbarPos("VALUE max", "HSV")

			#h_min=62
			#h_max=179
			#s_min=199
			#s_max=255
			#v_min=168
			#v_max=255
			
			lower=np.array([h_min,s_min,v_min])			
			upper=np.array([h_max,s_max,v_max])

			mask=cv2.inRange(image_array,lower,upper)
			result=cv2.bitwise_and(image_array,image_array, mask = mask)
			mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

			g=np.hstack([image_array,mask, result])
			#cv2.imshow("notmask", image_array)
			#cv2.imshow("hsv1", image_array_hsv)
			#cv2.imshow("mask", mask)
	
			cv2.imshow("result", g)

			if cv2.waitKey(1) & 0xFF==ord('q'):
				exit()
			#return img_with_lines




def main():
	rospy.init_node('remapping_node', anonymous=True)
	image_rec_obj=image_rec()	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down the program")
		

if __name__=='__main__':
	main()

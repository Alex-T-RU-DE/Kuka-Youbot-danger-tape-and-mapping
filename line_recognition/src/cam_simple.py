#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
from tape_coordinate.msg import tape_msgs
from tape_coordinate.msg import tape_msgs_array

class image_rec(object):
	
	
	def __init__(self):

		self.image_sub=rospy.Subscriber("/usb_cam/image_raw", Image, self.camera_callback)
		self.pub=rospy.Publisher('/line_coordinates', tape_msgs_array, queue_size=100)
		self.bridge_object=CvBridge()
	
	def camera_callback(self,data):
		try:
			cv_image=self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
			new_cv_image = self.recognition(cv_image)
		except CvBridgeError as e:
			print("Error")
        
	def recognition(self,image_array):
		
		#print("Yellow tape:")
		#imgContour, imgDil=self.imageOperation(image_array,1)

		#print("Blue tape:")
		#imgContour2, imgDil2=self.imageOperation(image_array,3)


		#imgStack=stackImages(0.8,([image_array, image_array, image_array],
		#				  [imgContour, imgContour1, imgContour2]))
		

		#print("Red tape:")
		imgContour1, imgDil1=self.imageOperation(image_array,2)
		cv2.imshow('frame',  imgContour1)	
		if cv2.waitKey(1) & 0xFF==ord('q'):
				exit()

	def imageOperation(self,image_array,color):
		imgContour=image_array.copy()
		image_blur=cv2.GaussianBlur(image_array, (7,7), 1)
		#yellow
		if color==1:
			lower=np.array([15,138,64])			
			upper=np.array([97,255,192])
		#red
		elif color==2:
			lower=np.array([0,0,191])			
			upper=np.array([79,255,255])
		#blue
		elif color==3:
			lower=np.array([59,0,0])			
			upper=np.array([136,50,65])
		mask=cv2.inRange(image_array,lower,upper)
		result=cv2.bitwise_and(image_array,image_array, mask = mask)
		imgCanny=cv2.Canny(mask,23,23)
		kernel=np.ones((5,5))
		imgDil=cv2.dilate(imgCanny, kernel, iterations=1)
		self.getContours(imgDil,imgContour)            
            
		return imgContour, imgDil
                           
            
	def getContours(self,img, imgContour):

		contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2:]
		msgobj_array=tape_msgs_array()
		for cnt in contours:
			area=cv2.contourArea(cnt)
			if area>300:
				msgobj=tape_msgs()
				cv2.drawContours(imgContour, cnt, -1, (0,255,0), 3)
				peri=cv2.arcLength(cnt, True)
				approx=cv2.approxPolyDP(cnt, 0.02*peri, True)
				x, y, w, h= cv2.boundingRect(approx)
				#print("coordinates: ")
				#print((x+w/2),(y+h/2))
				cv2.rectangle(imgContour, (x, y), (x+w, y+h), (255, 0, 0), 2)
				msgobj.posX=(x+w/2)
				msgobj.posY=(y+h/2)
				msgobj.length=0
				msgobj_array.tape_msgs_array.append(msgobj)
		self.pub.publish(msgobj_array)


def main():
	rospy.init_node('remapping_node', anonymous=True)
	image_rec_obj=image_rec()	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down the program")
		

if __name__=='__main__':
	main()

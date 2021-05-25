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

		#print("Red tape:")
		imgContour1, imgDil1=self.imageOperation(image_array,2)
		
		cv2.imshow('frame',  imgContour1)
		if cv2.waitKey(1) & 0xFF==ord('q'):
				exit()

	def imageOperation(self,image_array,color):
		imgContour=image_array.copy()
		image_blur=cv2.GaussianBlur(image_array, (3,3), 1)
		
		#yellow
		if color==1:
			lower=np.array([15,138,64])			
			upper=np.array([97,255,192])
		#red
		elif color==2:
			lower=np.array([0,0,255])			
			upper=np.array([105,255,255])
		#blue
		elif color==3:
			lower=np.array([59,0,0])			
			upper=np.array([136,50,65])
		elif color==4:
			lower=np.array([0,0,0])			
			upper=np.array([179,255,255])
		mask=cv2.inRange(image_array,lower,upper)
		result=cv2.bitwise_and(image_array,image_array, mask = mask)
		imgCanny=cv2.Canny(mask,23,23)
		kernel=np.ones((5,5))
		imgDil=cv2.dilate(imgCanny, kernel, iterations=1)
		self.getContours(imgDil,imgContour, image_array)            
            
		return imgContour, imgDil
            
                 
            
	def getContours(self,img, imgContour, image_array):

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
				
				###################
				if(x>10 and y>10):
					crop_img = image_array.copy()				
					crop_img = crop_img[y-10:y+h+10, x-10:x+w+10]
				else:
					crop_img = image_array.copy()
					crop_img = crop_img[y:y+h, x:x+w]
				if self.imageOperation_1(crop_img,2):
					#cv2.imshow('croped', crop_img)
					cv2.rectangle(imgContour, (x, y), (x+w, y+h), (255, 0, 0), 2)
					msgobj.posX=(x+w/2)
					msgobj.posY=(y+h/2)
					msgobj.length=0
					msgobj_array.tape_msgs_array.append(msgobj)

		self.pub.publish(msgobj_array)


	#the second step of image processing: we are considering each contour that we found before to increase the accuracy 
	def imageOperation_1(self,image_array,color):
		imgContour=image_array.copy()
		image_blur=cv2.GaussianBlur(image_array, (9,9), 1)
		#image_blur=cv2.bilateralFilter(image_array,9,75,75)
		#yellow
		if color==1:
			lower=np.array([15,138,64])			
			upper=np.array([97,255,192])
		#red
		elif color==2:
			lower=np.array([0,0,145])			
			upper=np.array([120,170,255])
		#blue
		elif color==3:
			lower=np.array([59,0,0])			
			upper=np.array([136,50,65])
		mask=cv2.inRange(image_array,lower,upper)
		result=cv2.bitwise_and(image_array,image_array, mask = mask)
		imgCanny=cv2.Canny(mask,23,23)
		kernel=np.ones((5,5))
		imgDil=cv2.dilate(imgCanny, kernel, iterations=1)
		logic=self.getContours_1(imgDil,imgContour)            
            
		return logic

	#this method checking each contour for the number of the end-points here.
	#In other words, we are checking whether its a rectangle(or something like a rectangle) or not 
	#it helps us to avoid the recognition of different objects with the same color as we are searching for
	def getContours_1(self,img, imgContour):

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
				if (len(approx)<=5 and len(approx)>=3):
					if(x!=0 and y!=0 and w!=0 and h!=0):
						return True
					else:
						return False
				else:
					return False



def main():
	rospy.init_node('remapping_node', anonymous=True)
	image_rec_obj=image_rec()	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down the program")
		

if __name__=='__main__':
	main()

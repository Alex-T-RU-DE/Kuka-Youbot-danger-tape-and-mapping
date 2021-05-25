#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import numpy as np
import rospy
import time
import tf
import tf2_ros
import math
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt

pub = rospy.Publisher('new_base_scan', LaserScan, queue_size=10)

def callback(data):

    new_scan=data
    ranges_arr=list(new_scan.ranges)
    for i in range(85):
		ranges_arr[i]=float("nan")
    for i in range(37):
    		ranges_arr[len(ranges_arr)-37+i]=float("nan")
    new_scan.ranges=tuple(ranges_arr)
    pub.publish(new_scan)
    	
def listener():
    
    rospy.init_node('scan_cleaning', anonymous=True)
    rospy.Subscriber("base_scan", LaserScan, callback)
    
    rospy.spin()

if __name__ == '__main__':
    listener()

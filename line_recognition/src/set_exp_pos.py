#!/usr/bin/env python  
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIKRequest, GetPositionIK
import time
import numpy as np
import sys


	
def moveToTravel2(group):
	joint_goal = group.get_current_joint_values()
	joint_goal[0] = 3.05
	joint_goal[1] = 0.01
	joint_goal[2] = -0.3
	joint_goal[3] = 2.65
	joint_goal[4] = 2.925
	group.set_joint_value_target(joint_goal)
	#plan2 = group.plan()
	group.go(wait=True)
	group.stop()
	group.clear_pose_targets()
	rospy.loginfo(joint_goal)


		
def goGrasp():
	
	arm_group = moveit_commander.MoveGroupCommander("arm_1")
	robot_commander = moveit_commander.RobotCommander()
	time.sleep(1)
	moveToTravel2(arm_group)

				
	
if __name__ == '__main__':
    try:
     moveit_commander.roscpp_initialize(sys.argv)
     rospy.init_node('searching_pos_node')

     goGrasp()
    except rospy.ROSInterruptException:
     rospy.loginfo("test finished.")

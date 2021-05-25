#!/usr/bin/env python  
import rospy
import moveit_commander
import math
import tf
import tf2_ros
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIKRequest, GetPositionIK
import time
import numpy as np
import sys
from grip_msg.msg import grip_msg

def getTF():
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    time.sleep(1)
    camtob = tfBuffer.lookup_transform('arm_camera_link', 'base_link', rospy.Time())
    btocam = tfBuffer.lookup_transform('base_link', 'arm_link_5', rospy.Time())
    
    q = [camtob.transform.rotation.x, camtob.transform.rotation.y, camtob.transform.rotation.z, camtob.transform.rotation.w]
    #z = tf.transformations.euler_from_quaternion(q, axes='sxyz')
    qtomx = tf.transformations.quaternion_matrix(q)
    ctb = np.array([
    [qtomx[0][0], qtomx[0][1], qtomx[0][2]],
    [qtomx[1][0], qtomx[1][1], qtomx[1][2]], 
    [qtomx[2][0], qtomx[2][1], qtomx[2][2]]
    ])
    
    btc = np.array([
    btocam.transform.translation.x, 
    btocam.transform.translation.y, 
    0])
    
    h = btocam.transform.translation.z + 0.12
    
    return ctb, btc, h

def getObjXYZ(x,y):
    cameraMatrix = np.array([
	[596.83, 0, 317.44],
	[0, 598.09, 244.35], 
	[0, 0, 1]
	])
	
    R,T,h = getTF()
    pix = np.array([x, y, 1])
    
    invM = np.linalg.inv(cameraMatrix)
    rel = invM.dot(pix)
    rotated = R.dot(rel)
    result = rotated*h+T
    
    return result

def moveToTravel(group):
	joint_goal = group.get_current_joint_values()
	joint_goal[0] = 2.95
	joint_goal[1] = 0.1
	joint_goal[2] = -1.61
	joint_goal[3] = 3.18
	joint_goal[4] = 2.925
	group.set_joint_value_target(joint_goal)
	#plan2 = group.plan()
	group.go(wait=True)
	group.stop()
	group.clear_pose_targets()
	rospy.loginfo(joint_goal)
	
def moveToTravel2(group):
	joint_goal = group.get_current_joint_values()
	joint_goal[0] = 2.95
	joint_goal[1] = 0.1
	joint_goal[2] = -1.14
	joint_goal[3] = 3.51
	joint_goal[4] = 2.925
	group.set_joint_value_target(joint_goal)
	#plan2 = group.plan()
	group.go(wait=True)
	group.stop()
	group.clear_pose_targets()
	rospy.loginfo(joint_goal)

def moveToTravel3(group):
	joint_goal = group.get_current_joint_values()
	joint_goal[0] = 2.83
	joint_goal[1] = 0.1
	joint_goal[2] = -3.52
	joint_goal[3] = 1.47
	joint_goal[4] = 2.81
	group.set_joint_value_target(joint_goal)
	plan2 = group.plan()
	group.go(wait=True)
	group.stop()
	group.clear_pose_targets()

def moveShift(group,axis,val):
	group.shift_pose_target(axis,val)
	
	group.go(wait=True)
	group.stop()
	group.clear_pose_targets()

def openGripper(pub):
	ms = grip_msg()
	ms.Procent = 25
	pub.publish(ms)
	time.sleep(4)
	
def grasp(pub):
	ms = grip_msg()
	ms.Procent = 0
	pub.publish(ms)
	time.sleep(4)
		
def gg(arm_group, robot_commander, xyz):
	
	compute_ik_srv = rospy.ServiceProxy("/compute_ik", GetPositionIK)
	rospy.wait_for_service("/compute_ik")
	
	pose = PoseStamped()
	pose.header.frame_id = "base_footprint"
	pose.header.stamp = rospy.Time.now()
	#pose.pose.orientation.x = -0.021
	#pose.pose.orientation.y = 0.957
	#pose.pose.orientation.z = -0.056
	#pose.pose.orientation.w = 0.283
	#pose.pose.position.x = 0.409
	#pose.pose.position.y = -0.043
	#pose.pose.position.z = 0.173
	
	pose.pose.orientation.x = 0
	pose.pose.orientation.y = 1
	pose.pose.orientation.z = 0
	pose.pose.orientation.w = 0
	pose.pose.position.x = xyz[0]
	pose.pose.position.y = xyz[1]
	pose.pose.position.z = 0.05
	req = GetPositionIKRequest()
	req.ik_request.group_name = "arm_1"
	req.ik_request.robot_state = robot_commander.get_current_state()
	req.ik_request.avoid_collisions = True
	req.ik_request.pose_stamped = pose
	ans = compute_ik_srv(req)
	
	rospy.loginfo(ans)
	joints = arm_group.get_current_joint_values()
	joints[0] = ans.solution.joint_state.position[0]
	joints[1] = ans.solution.joint_state.position[1]
	joints[2] = ans.solution.joint_state.position[2]
	joints[3] = ans.solution.joint_state.position[3]
	joints[4] = ans.solution.joint_state.position[4]
	arm_group.set_joint_value_target(joints)

	plan2 = arm_group.plan()
	arm_group.go(wait=True)
	#plan1 = arm_group.plan()
	
	#a = [0.409, -0.043, 0.173]
	#arm_group.set_position_target(a)
	#moveToTravel(arm_group)
	#arm_group.go(wait=True)
	#arm_group.stop()
	#arm_group.clear_pose_targets()
	
def goGrasp():
	
	arm_group = moveit_commander.MoveGroupCommander("arm_1")
	robot_commander = moveit_commander.RobotCommander()
	
	gripper_publisher = rospy.Publisher('Gripping', grip_msg, queue_size=10)
	time.sleep(1)
	openGripper(gripper_publisher)
	#grasp(gripper_publisher)
	moveToTravel2(arm_group)
	#time.sleep(2)
	#moveToTravel2(arm_group)
	#time.sleep(2)
	#moveToTravel3(arm_group)
	#xyz = getObjXYZ(150.0,335.5)
	#xyz = getObjXYZ(150.0,335.5)
	#rospy.loginfo(xyz)
	#gg(arm_group,robot_commander,xyz)
	
		
	
	
if __name__ == '__main__':
    try:
     moveit_commander.roscpp_initialize(sys.argv)
     rospy.init_node('tf2_listener')
    
     #gg()
     goGrasp()
    except rospy.ROSInterruptException:
     rospy.loginfo("test finished.")

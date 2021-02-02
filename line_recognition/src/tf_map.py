#!/usr/bin/env python  
import roslib
#roslib.load_manifest('learning_tf')

import rospy
import tf
import math

if __name__ == '__main__':
    rospy.init_node('dynamic_tf_map')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec() * math.pi
        br.sendTransform((-75, -75, 0.0),
                         (0.0, 0.0, 0.0, 1),
                         rospy.Time.now(),
                         "new_map",
                         "odom")
        rate.sleep()

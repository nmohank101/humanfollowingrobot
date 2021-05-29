#!/usr/bin/env python  
import rospy
import tf
import tf2_ros
import math
import geometry_msgs.msg
import numpy as np
from geometry_msgs.msg import Twist

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

def cmdstop():
    print "stopping"
    cmd = Twist()
    cmd.angular.z = 0
    cmd.linear.x = 0
    pub.publish(cmd)   

def generateVelCommand(x, theta):
    cmd = Twist()
    cmd.angular.z = theta *1
    cmd.linear.x = (x-1.2*np.cos(theta))*0.25       # Distance to keep 0.5m
    pub.publish(cmd)


if __name__ == '__main__':
    rospy.init_node('pub_cmdVel')
    cmdstop()
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(100.0)
    count = 0
    while not rospy.is_shutdown():
        try:
            trans= tfBuffer.lookup_transform('openni_depth_frame', 'followme', rospy.Time())
            print(trans)
            #rospy.loginfo("Distance between camera and person = {0:f}".format(np.linalg.norm(trans)))
            rospy.loginfo("Angle = {0:f}".format(np.arctan2(trans.transform.translation.y, trans.transform.translation.x) * 180 / np.pi))
            if (trans.transform.translation.x<5):
                generateVelCommand(trans.transform.translation.x,np.arctan2(trans.transform.translation.y, trans.transform.translation.x))
            else:
                cmdstop()
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #        cmdstop()
            continue

        #rate.sleep()

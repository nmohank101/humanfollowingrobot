#!/usr/bin/env python
import dbscan
import rospy
import tf
import tf2_ros
import geometry_msgs
from fiducial_msgs.msg import FiducialTransformArray
from geometry_msgs.msg import TransformStamped
from tf.msg import tfMessage


def callback(data):
     br = tf2_ros.TransformBroadcaster()
     t = geometry_msgs.msg.TransformStamped()
        
 
     if len(data.transforms)!=0:
      for i in range(len(data.transforms)):
        

          t.header.stamp = data.header.stamp #Update this
          t.header.frame_id = data.header.frame_id
   
          t.child_frame_id = str(data.transforms[i].fiducial_id)
          t.transform = data.transforms[i].transform
        
          br.sendTransform(t)

def publisher():
    rospy.init_node('pub_arucotf3', anonymous=True)
    rospy.Subscriber("fiducial_transforms",FiducialTransformArray,callback)
    rospy.spin()
    
  
if __name__ == '__main__':
  publisher()

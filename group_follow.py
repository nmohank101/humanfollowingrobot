#!/usr/bin/env python
import dbscan
import statistics
import numpy as np
import rospy
import tf
import tf2_ros
import geometry_msgs
from fiducial_msgs.msg import FiducialTransformArray
from geometry_msgs.msg import TransformStamped
from tf.msg import tfMessage
from scipy.stats import mode
from transforms3d.quaternions import rotate_vector
from transforms3d.euler import euler2mat

# marker_available = 0
data1 = 0

def create_transform(t,trans):
  t.header.stamp = rospy.Time.now()
  t.transform.translation.x =trans.transform.translation.x
  t.transform.translation.y =trans.transform.translation.y
  t.transform.translation.z =trans.transform.translation.z
  t.transform.rotation.x = trans.transform.rotation.x
  t.transform.rotation.y = trans.transform.rotation.y
  t.transform.rotation.z = trans.transform.rotation.z
  t.transform.rotation.w = trans.transform.rotation.w

def callback(data):
  global data1
  global marker_available
  data1=data
  marker_available=1


if __name__ == '__main__':
  rospy.init_node('group_follow')
  tfBuffer = tf2_ros.Buffer(rospy.Duration(0.1))
  listener2 = tf2_ros.TransformListener(tfBuffer)
  br = tf2_ros.TransformBroadcaster()
  t = geometry_msgs.msg.TransformStamped()
  t.header.frame_id = "openni_depth_frame"
  t.child_frame_id = "followme"

  # To save Last Frame
  closest_torso = "openni_depth_frame"

  r= rospy.Rate(10.0)

  max_toro = 20
  case = 0

  global marker_available
  marker_available = 0
  torso_available = 0

  #index_torso = []
  index_mtorso = []

  iter_torso = 1

  marker_torso_transforms = []

  default_transform = geometry_msgs.msg.TransformStamped()
  default_transform.header.frame_id = "openni_depth_frame"
  default_transform.child_frame_id = "followme"
  default_transform.transform.translation.x=1.2
  default_transform.transform.translation.y=0
  default_transform.transform.translation.z=0
  default_transform.transform.rotation.x=0
  default_transform.transform.rotation.y=0
  default_transform.transform.rotation.z=0
  default_transform.transform.rotation.w=1

  closest_torso_transform = default_transform

  while not rospy.is_shutdown():
    distance_array=[]
    marker_torso_transforms = []
    torso_array=[]
    iter_torso=1

    rospy.Subscriber("fiducial_transforms",FiducialTransformArray,callback)

    check_marker = marker_available
    marker_available = 0

    
    # Check for torso
    torso_count = 0
    torso_trans_array =[]

    while(iter_torso<max_toro):
      torso = "torso_" + str(iter_torso)
      try:
        trans_torso =tfBuffer.lookup_transform('openni_depth_frame',torso,rospy.Time.now(),rospy.Duration(0.05))
        torso_trans_array.append(trans_torso)
        torso_array.append(torso)
        torso_available = 1
        torso_count = torso_count+1
      except:
          torso_available = 0
      iter_torso=iter_torso+1
    if torso_count == 0:
        rospy.loginfo('No people in frame to follow .. ! Waiting to see people ..')
    else:
        rospy.loginfo('People Detected:'+str(torso_count))


    data2 = data1

    if check_marker == 1:
    	r=[]
        noof100=0
        if len(data2.transforms)!=0:
	         for i in range(len(data2.transforms)):
		          if(str(data2.transforms[i].fiducial_id)=="100"):
			           noof100=noof100+1;
			           R = np.array(euler2mat(1.571,-1.571,0)).T
			           t_c = np.array([-0.045,0,0]).T
			           p_marker = np.array([data2.transforms[i].transform.translation.x, data2.transforms[i].transform.translation.y, data2.transforms[i].transform.translation.z]).T
			           p_marker_new = R.dot(p_marker)+t_c 
			           r.append(p_marker_new)
		           
		          #print data.transforms[i]
        r=np.array(r)      
        r=r.T
        bimod=[]
        if(noof100!=0):
		          k=dbscan.dbscan(r, 1.0, 1)  # 1.5m is costomizable
		          a=np.zeros(len(k))
		          for i in range(len(k)):
		            	a[k[i]-1]=a[k[i]-1]+1

		          min=-1
		          for i in range(len(k)):
		          	if(a[i]>min):
		          		min=a[i]
		          		mode=i+1
		          for i in range(len(k)):
		          	if(a[i]==min):
		          		bimod.append(i+1)
		          norm=100
		          for i in (bimod):
			      	  mean1=0
			      	  mean2=0
			      	  for j in range(len(k)):
			      	  	if(k[j]==i):
			      	  		mean1=r[0][j]+mean1
			      	  		mean2=r[1][j]+mean2
			      	  mean=mean1*mean1+mean2*mean2
			      	  if(mean<norm):
			      	  	norm=mean
			      	  	mode=i
			      	  	print mode

			       	    	

			              	
		          mean=[0,0,0]  
		          j=0  
		          for i in range(len(k)):
			            if(k[i]==mode):
				              mean[0]=mean[0]+r[0][i]
				              mean[1]=mean[1]+r[1][i]
				              mean[2]=mean[2]+r[2][i]
				              j=j+1

		          mean[0]=(mean[0])/(j)
		          mean[1]=(mean[1])/(j) 
		          mean[2]=mean[2]/j

		          t.header.stamp = rospy.Time.now()
		          t.transform.translation.x = mean[0]
		          t.transform.translation.y = mean[1]
		          t.transform.translation.z = mean[2]
		          t.transform.rotation.x = 0
		          t.transform.rotation.y = 0 
		          t.transform.rotation.z = 0
		          t.transform.rotation.w = 1
		          br.sendTransform(t)

	print noof100

	if torso_count !=0 and noof100 != 0:
		case = 1
		distance_array=[]
		marker_torso_transforms = []
		for t_trans in torso_trans_array:
			dist = (t_trans.transform.translation.x-mean[0])**2 + (t_trans.transform.translation.y-mean[1])**2
			distance_array.append(dist)
		distance_array=np.array(distance_array)
		closest_torso = torso_array[np.argmin(distance_array)]
		rospy.loginfo('Both Available '+'Case = '+str(case) + 'Closest Available Torso : '+ closest_torso) 
	
	elif torso_count == 0 and noof100 ==0:
		case = 0
		default_transform.header.stamp=rospy.Time.now()
		br.sendTransform(default_transform)
		rospy.loginfo('None Available '+'Case = '+str(case))

	elif torso_count ==0 and noof100 !=0:
		case = 2
		rospy.loginfo('Marker Available, Torso Unavailable '+'Case = '+str(case))

	elif torso_count !=0 and noof100 ==0:
		if case == 1:
			try:
				trans=tfBuffer.lookup_transform('openni_depth_frame',closest_torso,rospy.Time.now(),rospy.Duration(0.05))
				create_transform(t,trans)
				br.sendTransform(t)
				rospy.loginfo('Marker Unvailable, Torso Available '+'Case = '+str(44))
			except:
				rospy.loginfo('No Transform to closest_torso')
		else:
			default_transform.header.stamp=rospy.Time.now()
			br.sendTransform(default_transform)



        

#!/usr/bin/env python
from sslib import *
import sys
from visualuwb.srv import Rendezvous
from geometry_msgs.msg import Pose
import tf

if __name__ == '__main__':
    
    rospy.init_node('controller', anonymous=True)
    
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    msg = Twist()
    msg.linear.x = 0
    msg.linear.y = 0
    msg.linear.z = 0.2
    msg.angular.z = 0
    pub.publish(msg)
    

    rate = rospy.Rate(10)
    rospy.sleep(2)
    for i in xrange(20):
        pub.publish(msg)
        rate.sleep()
    
    msg.linear.z = 0
    pub.publish(msg)
        
    print "bhaha"
    rospy.wait_for_service('rendezvous_service')
    hunt = rospy.ServiceProxy('rendezvous_service', Rendezvous)
    
    
    listener = tf.TransformListener()
    
    while not rospy.is_shutdown():
        try:
            pose0,pose1,pose2,pose3 = Pose(),Pose(),Pose(),Pose()
            (T0, Q0) = listener.lookupTransform('/world', '/target/base_stabilized', rospy.Time(0))       
            (T1, Q1) = listener.lookupTransform('/world', "/"+sys.argv[1]+'/base_stabilized', rospy.Time(0))
            (T2, Q2) = listener.lookupTransform('/world', "/"+sys.argv[2]+'/base_stabilized', rospy.Time(0))
            (T3, Q3) = listener.lookupTransform('/world', "/"+sys.argv[3]+'/base_stabilized', rospy.Time(0))
            
            pose0.position.x,  pose0.position.y,  pose0.position.z = T0[0], T0[1], T0[2]
            pose0.orientation.x, pose0.orientation.x, pose0.orientation.x, pose0.orientation.x = Q0[0], Q0[1], Q0[2], Q0[3]
            
            pose1.position.x,  pose1.position.y,  pose1.position.z = T1[0], T1[1], T1[2]
            pose1.orientation.x, pose1.orientation.x, pose1.orientation.x, pose1.orientation.x = Q1[0], Q1[1], Q1[2], Q1[3]
            
            pose2.position.x,  pose2.position.y,  pose2.position.z = T2[0], T2[1], T2[2]
            pose2.orientation.x, pose2.orientation.x, pose2.orientation.x, pose2.orientation.x = Q2[0], Q2[1], Q2[2], Q2[3]
            
            pose3.position.x,  pose3.position.y,  pose3.position.z = T3[0], T3[1], T3[2]
            pose3.orientation.x, pose3.orientation.x, pose3.orientation.x, pose3.orientation.x = Q3[0], Q3[1], Q3[2], Q3[3]
            
            poses = []
            poses.append(pose0)
            poses.append(pose1)
            poses.append(pose2)
            poses.append(pose3)
            #print poses
            res = hunt(poses)
            pub.publish(res.twist[1])
            rate.sleep()
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "cant't get translation rotation or can't get controller or can not pub control information"
               
        
        
        
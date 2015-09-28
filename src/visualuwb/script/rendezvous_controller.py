#!/usr/bin/env python
from sslib import *
import sys
from visualuwb.srv import Rendezvous
from geometry_msgs.msg import Pose

global poses

def selfcallback(msg):
    global poses
    #print sys.argv[1]+sys.argv[2]+"self"
    poses[1] = msg.pose

def targetback(msg):
    global poses
    #print sys.argv[1]+sys.argv[2]+"neb2"
    poses[0] = msg.pose
   
def neigh1back(msg):
    global poses
    #print sys.argv[1]+sys.argv[2]+"neb1"
    poses[2] = msg.pose

    
def neigh2back(msg):
    global poses
    #print sys.argv[1]+sys.argv[2]+"target"
    poses[3] = msg.pose



if __name__ == '__main__':
    
    global poses
    poses = []
    for i in xrange(4):
        p = Pose()
        p.orientation.w = 1
        poses.append(Pose()) 
    
    rospy.init_node('controller', anonymous=True)
    
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    msg = Twist()
    msg.linear.x = 0
    msg.linear.y = 0
    msg.linear.z = 0.2
    msg.angular.z = 0
    pub.publish(msg)
    
    rospy.Subscriber("ground_truth_to_tf/pose", PoseStamped, selfcallback)
    rospy.Subscriber("/target/ground_truth_to_tf/pose", PoseStamped, targetback)
    rospy.Subscriber("/"+sys.argv[1]+"/ground_truth_to_tf/pose", PoseStamped, neigh1back)
    rospy.Subscriber("/"+sys.argv[2]+"/ground_truth_to_tf/pose", PoseStamped, neigh2back)
    
    #rospy.sleep(2)
    for i in xrange(20):
        pub.publish(msg)
    
    rospy.wait_for_service('rendezvous_service')
    hunt = rospy.ServiceProxy('rendezvous_service', Rendezvous)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        res = hunt(poses)
        pub.publish(res.twist[1])
        rate.sleep()
        
        
        
        
        
        
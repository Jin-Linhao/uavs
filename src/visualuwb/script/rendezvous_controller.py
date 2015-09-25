#!/usr/bin/env python
from sslib import *

 
if __name__ == '__main__':
    
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.init_node('controller', anonymous=True)
    rate = rospy.Rate(10)
    msg = Twist()
    msg.linear.x = 0
    msg.linear.y = 0
    msg.linear.z = 0.2
    msg.angular.z = 0
      
 
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()

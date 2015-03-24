#!/usr/bin/env python

import sys
import rospy
from director.srv import *
from geometry_msgs.msg import Point

def require():
    rospy.wait_for_service('/director/prenode')

    try:
        nextnode = rospy.ServiceProxy('/director/prenode', PreNode)
        pre      = nextnode(1)
        print  pre
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == "__main__":
    rospy.init_node('require', anonymous=True)   
    while not rospy.is_shutdown():
        require()

#!/usr/bin/env python
from sslib import *
from visualuwb.srv import *
from HuntController import *

        
def server(req):
    #print "receiving new requirement!"
    for pose in req.pose:
        print pose
        
    hunt = HuntController()
    
    res = hunt.decide(req.pose)
    
    return res
if __name__ == '__main__':
    
    rospy.init_node('rendezvous_server', anonymous = True)
    
    s = rospy.Service('rendezvous_service', Rendezvous, server)
    
    print "Ready to serve for rendezvous server"
    
    rospy.spin()
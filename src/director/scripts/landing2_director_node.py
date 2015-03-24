#!/usr/bin/env python
import os
import rospy
from std_msgs.msg import String
from director.msg import ExitNode
from director.srv import *
import rospy
nodes = NextNode()

def handle_nextnode(req):
    print "receive request from nodes..."
    #print req
    global nodes
    nodes =req
    return 1

def handle_prenode(confirm):
    global nodes

    next_node =nodes.next_node
    area   = nodes.area[:]
    length = nodes.length[:]
    width  = nodes.width[:]
    hight  = nodes.hight[:]
    pos    = nodes.pos[:]

    prenode =[]
    prenode.append(next_node)
    prenode.append(area)
    prenode.append(length)
    prenode.append(width)
    prenode.append(hight)
    prenode.append(pos)
    #print prenode
    return prenode


if __name__ == '__main__':

    pub = rospy.Publisher('exit_node', ExitNode, 1)
    rospy.init_node('director', anonymous=True)
    nodemsg = ExitNode()
    nodemsg.node_name = "sample_node"
    rate = rospy.Rate(10)

    s = rospy.Service('/director/nextnode', NextNode, handle_nextnode)
    print "Ready for serving for next node ..."

    s = rospy.Service('/director/prenode', PreNode, handle_prenode)
    print "Ready for serving for pre node ..."

    global nodes

    nodes.next_node='landing2_node'
    while not rospy.is_shutdown() and nodes.next_node!='':
        print 'going to start node-' + nodes.next_node
        launchcmd = 'roslaunch path '+ nodes.next_node + '.launch'
        os.system(launchcmd)

    #print "Ready for exiting nodes ..."
    #while not rospy.is_shutdown():
    #    print nodes
    #    rospy.loginfo(nodemsg.node_name)
    #    pub.publish(nodemsg)
    #    rate.sleep()









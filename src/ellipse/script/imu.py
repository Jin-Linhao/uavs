#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from subprocess import Popen,PIPE,signal
import os,sys
import time

def RUN():
    sp = Popen(('../src/Examples/ellipseMinimal/ellipseMinimal /dev/ttyUSB0'),bufsize=0, stdout=PIPE,shell= True)
    pub = rospy.Publisher('/fcu/imu/data', Imu, queue_size=1)
    rospy.init_node('ExternalImu', anonymous=True)
    mes = Imu()
    quat = 0
    imu  = 0
    while not rospy.is_shutdown():
	data = sp.stdout.readline()
	words = data.split(" ")
	if words[0]=='QUAT':
		mes.orientation.x=float(words[2])
		mes.orientation.y=float(words[3])
		mes.orientation.z=float(words[4])
		mes.orientation.w=float(words[5])
		quat = 1	
	elif words[0]=='IMU':
		mes.angular_velocity.x=float(words[10])
		mes.angular_velocity.y=float(words[11])
		mes.angular_velocity.z=float(words[12])
		mes.linear_acceleration.x=float(words[3])
		mes.linear_acceleration.y=float(words[4])
		mes.linear_acceleration.z=float(words[5])
                imu  = 1
	elif words[0]=='POSE':
		pass
	elif words[0]=='EULER':
		pass
	if quat ==1 and imu == 1:
		mes.header.stamp = rospy.Time.now()
		quat = 0
		imu  = 0
		pub.publish(mes)
		
if __name__ == '__main__':
    try:
    	RUN()
    except rospy.ROSInterruptException:
        pass


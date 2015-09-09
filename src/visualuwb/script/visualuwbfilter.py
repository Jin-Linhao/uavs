#!/usr/bin/env python
__author__ = 'Jeffsan'
from pykalman import UnscentedKalmanFilter, AdditiveUnscentedKalmanFilter
from numpy.oldnumeric.linear_algebra import inverse
from matplotlib.lines import lineStyles
from mpl_toolkits.mplot3d import Axes3D
from scipy.integrate import odeint
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from pykalman import KalmanFilter
import matplotlib.pyplot as plt
from filter import *
from numpy import *
import time
import copy
icount = 0
anchor = array([[-2,-2,0.8],[-2,2,1.2],[2,2,1.3],[2,-2,0.2]])
N = 100
x = array([[i, i, i*0.4, 0, 0, 0, 4.0/N, 4.0/N, 0.4*4.0/N, 0] for i in linspace(0,4,N)])   
y = array([[linalg.norm(x[i,0:3]-anchor[i%4])] for i in xrange(0,N)])
noise = random.randn(N,1)*0.1
measure = y + noise
state_estimation = zeros((1,10))[0]
state_estimation[2]=0.27
curent_u = tuple([[0,0,-g,0,0,0]])
uwb = UWBLocation() 
imu = IMULocation() 


def statecallback(msg):
    global icount
    global state_estimation
   
    pos        = array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
    anchor_pos = anchor[icount%4]
    icount     = icount+1
    y          = linalg.norm(pos - anchor_pos)#+ random.randn(1,1)[0]*0.1
    q          = msg.pose.pose.orientation
    vels       = msg.twist.twist.linear
    vel        = array([vels.x, vels.y, vels.z])
    #state_estimation[6:9] = vel
    #state_estimation[3:6] = euler_from_quaternion((q.x,q.y,q.z,q.w))

    (state_estimation, p)=uwb.locate(state_estimation, Q, 1.0/(100), y, anchor_pos)
    
    br  = tf.TransformBroadcaster()
    uwb_tran = state_estimation[0:3]
    uwb_q = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w] 
    br.sendTransform(uwb_tran,uwb_q, rospy.Time.now(), "uwb","world")  
    print state_estimation[0:3],pos


def imucallback(msg):
    global state_estimation
    q     = (msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w)
    euler = array(euler_from_quaternion(q))
    rate  = array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.x])
    acc   = array([msg.linear_acceleration.x,msg.linear_acceleration.y, msg.linear_acceleration.z])
    #(state_estimation, p)=imu.locate(state_estimation, Q, 1.0/(100/1), euler, acc, rate)
    #br = tf.TransformBroadcaster()
    #br.sendTransform(uwb_tran,(0,0,0,1), rospy.Time.now(), "uwb","world")  



if __name__ == '__main__':
    
    test_module='bagtext'
    if test_module == 'bagtext':
        f = open('ground_truth.txt','r')
        l = array([ map(float,line.split(' ')) for line in f if line.strip() != "" ])      
        N = 800#l.shape[0]/8

        p, qr, v, a, r, q  = l[0:N,2:5], l[0:N,5:9], l[0:N,9:12], l[0:N,19:22], l[0:N,22:25],l[0:N,15:19]
        #a[:,0],a[:,1],a[:,2]= -a[:,0],-a[:,1],-a[:,2]
        e = array([euler_from_quaternion(q[i]) for i in xrange(0,N)])
        #for i in xrange(1,N):
        #    print linalg.norm(qr[i]-q[i])
        #exit(0)


        #x = array([[i, i, i*0.4, 0, 0, 0, 4.0/N, 4.0/N, 0.4*4.0/N, 0] for i in linspace(0,4,N)])   
        y = array([[linalg.norm(p[i]-anchor[i%4])] for i in xrange(0,N)])
        n = random.randn(N,1)*0.1
        measure = y #+ n
        
        xe = zeros((N,10))
        xe[0,2] = 0.27
        uwb = UWBLocation()
        for i in xrange(0, N-1):
            xe[i+1], pp = uwb.locate(xe[i], Q, 1.0/100, measure[i,0], anchor[i%4], e[i], a[i], r[i])
            print i








        #plot data
        fig = plt.figure()
        ax = fig.add_subplot(331, projection='3d')
        ax.plot(anchor[:,0],anchor[:,1],anchor[:,2],marker='o',linewidth=3)
        ax.plot(xe[:,0], xe[:,1], xe[:,2])
        ax.plot(p[:,0], p[:,1], p[:,2])

        ax = fig.add_subplot(333)
        ax.plot(abs(xe[:,0]-p[:,0]),color = 'red')
        ax.plot(abs(xe[:,1]-p[:,1]),color = 'blue')
        ax.plot(abs(xe[:,2]-p[:,2]),color = 'black')
        plt.title('error of position')

        ax = fig.add_subplot(332)
        ax.plot(v[:,0],color = 'red')
        ax.plot(v[:,1],color = 'blue')
        ax.plot(v[:,2],color = 'black') 
        plt.title('real vel')

        ax = fig.add_subplot(6,1,3)
        ax.plot(a[:,0],color = 'red')
        ax.plot(a[:,1],color = 'blue')
        ax.plot(a[:,2],color = 'black') 
        plt.title('real acc')

        ax = fig.add_subplot(6,1,4)
        ax.plot(r[:,0],color = 'red')
        ax.plot(r[:,1],color = 'blue')
        ax.plot(r[:,2],color = 'black') 
        plt.title('real rate')

        ax = fig.add_subplot(6,1,5)
        ax.plot(xe[:,6],color = 'red')
        ax.plot(xe[:,7],color = 'blue')
        ax.plot(xe[:,8],color = 'black') 
        plt.title('estimated euler')

        ax = fig.add_subplot(6,1,6)
        ax.plot(e[:,0],color = 'red')
        ax.plot(e[:,1],color = 'blue')
        ax.plot(e[:,2],color = 'black') 
        plt.title('real euler')


        plt.show()




    if test_module == 'rviz':
        import rospy
        import tf
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu
        from tf.transformations import euler_from_quaternion
        from tf.transformations import quaternion_from_euler
        rospy.init_node('uav_filter')
        rate = rospy.Rate(30.0)
        rospy.Subscriber("/ground_truth/state", Odometry, statecallback)
        rospy.Subscriber("/raw_imu", Imu, imucallback)
        rospy.spin()
        
    
    elif test_module=='imu':
        imu = IMULocation()   
        xe = zeros(x.shape)
        p  = zeros((N,x.shape[1],x.shape[1]))
    
        start = time.time()
        for i in xrange(0, N-1):
            xe[i+1], p[i+1] = imu.locate(xe[i], Q, 1.0*t/N, measure[i], anchor[i%4])
           
        end = time.time()
        print (end - start)/N
    
        fig = plt.figure()
        ax = fig.add_subplot(121, projection='3d')
        ax.plot(anchor[:,0],anchor[:,1],anchor[:,2],marker='o',linewidth=3)
        ax.plot(xe[:,0], xe[:,1], xe[:,2])
        ax.plot(x[:,0], x[:,1], x[:,2])
    
        ax = fig.add_subplot(222)
        ax.plot(abs(noise), color = 'black',linewidth=2.5)
        ax.plot(abs(xe[:,0]-x[:,0]),color = 'red')
        ax.plot(abs(xe[:,1]-x[:,1]),color = 'blue')
        ax.plot(abs(xe[:,2]-x[:,2]),color = 'black')
        
        
        ax = fig.add_subplot(224)
        ax.plot(abs(xe[:,6]-x[:,6]),color = 'red')
        ax.plot(abs(xe[:,7]-x[:,7]),color = 'blue')
        ax.plot(abs(xe[:,8]-x[:,8]),color = 'black') 


    elif test_module=='uwb':
        uwb = UWBLocation()   
        xe = zeros(x.shape)
        p  = zeros((N,x.shape[1],x.shape[1]))
    
        start = time.time()
        for i in xrange(0, N-1):
            xe[i+1], p[i+1] = uwb.locate(xe[i], Q, 1.0*t/N, measure[i], anchor[i%4])
           
        end = time.time()
        print (end - start)/N
    
        fig = plt.figure()
        ax = fig.add_subplot(121, projection='3d')
        ax.plot(anchor[:,0],anchor[:,1],anchor[:,2],marker='o',linewidth=3)
        ax.plot(xe[:,0], xe[:,1], xe[:,2])
        ax.plot(x[:,0], x[:,1], x[:,2])
    
        ax = fig.add_subplot(222)
        ax.plot(abs(noise), color = 'black',linewidth=2.5)
        ax.plot(abs(xe[:,0]-x[:,0]),color = 'red')
        ax.plot(abs(xe[:,1]-x[:,1]),color = 'blue')
        ax.plot(abs(xe[:,2]-x[:,2]),color = 'black')
        
        
        ax = fig.add_subplot(224)
        ax.plot(abs(xe[:,6]-x[:,6]),color = 'red')
        ax.plot(abs(xe[:,7]-x[:,7]),color = 'blue')
        ax.plot(abs(xe[:,8]-x[:,8]),color = 'black')
        plt.show()

#===============================================================================
# #else:
# 
#     uwb = UWBLocation()
#     
#     N = 100
# 
#     x = array([[i, i, i*0.4, 4.0/N, 4.0/N, 0.4*4.0/N] for i in linspace(0,4,N)])
#     
#     anchor = array([[-0.1,-0.1,0.8],[4,0,1.3],[4,4,0.2],[0,4,1.2]])
#     
#     y = array([[linalg.norm(x[i,0:3]-anchor[i%4])] for i in xrange(0,N)])
# 
#     measure = y + random.randn(N,1)*0.1
# 
#     xe = zeros(x.shape)
#     p  = zeros((N,x.shape[1],x.shape[1]))
# 
#     for i in xrange(0, N-1):
#         xe[i+1], p[i+1] = uwb.locate(measure[i], anchor[i%4])
# 
#     t = linspace(0, 10, N) 
# 
#     fig = plt.figure()
#     ax = fig.add_subplot(121, projection='3d')
#     ax.plot(anchor[:,0],anchor[:,1],anchor[:,2],marker='o',linewidth=3)
#     ax.plot(xe[:,0], xe[:,1], xe[:,2])
#     ax.plot(x[:,0], x[:,1], x[:,2])
# 
#     ax = fig.add_subplot(222)
#     ax.plot(abs(xe[:,0]-x[:,0]),color = 'red')
#     ax.plot(abs(xe[:,1]-x[:,1]),color = 'blue')
#     ax.plot(abs(xe[:,2]-x[:,2]),color = 'black')
#    
#     
#     ax = fig.add_subplot(224)
#     ax.plot(abs(xe[:,3]),color = 'red')
#     ax.plot(abs(xe[:,4]),color = 'blue')
#     ax.plot(abs(xe[:,5]),color = 'black')
# 
#     plt.show()
#===============================================================================


    # vkf = KalmanFilter(n_dim_obs = 2, n_dim_state = 2,
    #                    transition_matrices  = [[1, 0], [0, 1]],
    #                    observation_matrices = [[1, 0], [0, 1]])
    #
    # n = 100
    # t = linspace(0, 2*pi, n)
    #
    # x = vstack((sin(t), cos(t))).T
    # y = x + 0.3*random.rand(n,2)
    # xe = zeros((n,2))
    # q = zeros((n,2,2))
    #
    # for i in range(0, n-1):
    #     (xe[i+1], q[i+1]) = vkf.filter_update(x[i],q[i], y[i])
    #
    # plt.subplot(121)
    # plt.plot(t, x[:,0], color='blue',   linewidth=2.5)
    # plt.plot(t, xe[:, 0], color='black',linewidth=2.5)
    # plt.plot(t, y[:, 0], color='green', linewidth=2.5)
    # plt.subplot(122)
    # plt.plot(t, x[:,1], color='blue', linewidth=2.5)
    # plt.plot(t, xe[:, 1], color='black',linewidth=2.5)
    # plt.plot(t, y[:, 1], color='green',linewidth=2.5)
    # plt.show()
    # print linalg.norm(xe-x), linalg.norm(y-x)


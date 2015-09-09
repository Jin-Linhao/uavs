#!/usr/bin/env python
from sslib import *
from VU_filter import *


if __name__ == '__main__':
    
    f = open('ground_truth.txt','r')
    l = array([ map(float,line.split(' ')) for line in f if line.strip() != "" ])
    #l = l[0:-1:2,:]
    N = 1000
    p, qr, v, a, r, q  = l[0:N,2:5], l[0:N,5:9], l[0:N,9:12], l[0:N,19:22], l[0:N,22:25],l[0:N,15:19]
    #a[:,0],a[:,1]= -a[:,0], -a[:,1]
    
    y = array([[linalg.norm(p[i]-anchor[i%4])] for i in xrange(0,N)])
    n = random.randn(N,1)*0.1
    measure = y + n
    xe = zeros((N,11))
    xe[0,2] = 0.27

    uwb = UWBLocation() 
    for i in xrange(0, N-1):
        xe[i+1], pp = uwb.locate(xe[i], Q, 1.0/100, measure[i,0], anchor[i%4], q[i], a[i], r[i])
        print i



    #plot data
    fig1 = plt.figure()
    ax = fig1.add_subplot(121, projection='3d')
    ax.plot(anchor[:,0],anchor[:,1],anchor[:,2],marker='o',linewidth=3)
    ax.plot(xe[:,0], xe[:,1], xe[:,2])
    ax.plot(p[:,0], p[:,1], p[:,2])
    
    ax = fig1.add_subplot(122)
    ax.plot(abs(xe[:,0]-p[:,0]),color = 'red')
    ax.plot(abs(xe[:,1]-p[:,1]),color = 'blue')
    ax.plot(abs(xe[:,2]-p[:,2]),color = 'black')
    plt.title('error of position')
    
    
    fig = plt.figure()
    ax = fig.add_subplot(321)
    ax.plot(v[:,0],color = 'red')
    ax.plot(v[:,1],color = 'blue')
    ax.plot(v[:,2],color = 'black') 
    plt.title('real vel')
    ax = fig.add_subplot(323)
    ax.plot(xe[:,7],color = 'red')
    ax.plot(xe[:,8],color = 'blue')
    ax.plot(xe[:,9],color = 'black') 
    plt.title('est vel')

    ax = fig.add_subplot(3,2,2)
    ax.plot(q[:,0],color = 'red')
    ax.plot(q[:,1],color = 'blue')
    ax.plot(q[:,2],color = 'black')
    ax.plot(q[:,3],color = 'green') 
    plt.title('real quaternion')
    ax = fig.add_subplot(3,2,4)
    ax.plot(xe[:,3],color = 'red')
    ax.plot(xe[:,4],color = 'blue')
    ax.plot(xe[:,5],color = 'black')
    ax.plot(xe[:,6],color = 'green') 
    plt.title('est quaternion')

    ax = fig.add_subplot(3,2,5)
    ax.plot(r[:,0],color = 'red')
    ax.plot(r[:,1],color = 'blue')
    ax.plot(r[:,2],color = 'black') 
    plt.title('real rate')

    ax = fig.add_subplot(3,2,6)
    ax.plot(a[:,0],color = 'red')
    ax.plot(a[:,1],color = 'blue')
    ax.plot(a[:,2],color = 'black') 
    plt.title('real acc')

    plt.show()

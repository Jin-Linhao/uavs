#!/usr/bin/env python
from matplotlib.lines import lineStyles
__author__ = 'Jeffsan'
from numpy import *
from pykalman import KalmanFilter
from pykalman import UnscentedKalmanFilter,AdditiveUnscentedKalmanFilter
#import matplotlib as plt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class FastVisionLocation:
    def __init__(self):
        self.M = 2
        self.N = 3
        self.A = [[1, 0, 0], [0, 1, 0],[0, 0, 1]]
        self.C = [[1, 0, 0], [0, 1, 0]]
        kf = KalmanFilter(n_dim_obs = self.M, n_dim_state = self.N,
                          transition_matrices=self.A, observation_matrices=self.C)

class UWBLocation:
    def __init__(self):
        self.N = 6
        self.M = 1
        self.x = zeros((1,self.N))[0]
        self.Q = 0.06*eye(self.N) 
        self.Q[-3:,-3:] = 0.1*self.Q[-3:,-3:]
        self.R = eye(1)*0.1
        self.A = eye(self.N) + diag(ones((1,3))[0],3)
        self.ukfinit()
        
    def ukfinit(self):

        self.ukf = UnscentedKalmanFilter(n_dim_obs = self.M, n_dim_state = self.N,
                                        transition_functions     = self.transition_function,
                                        observation_functions    = self.observation_function,
                                        transition_covariance    = self.Q,
                                        observation_covariance   = self.R,
                                        initial_state_mean       = self.x,
                                        initial_state_covariance = self.Q)

    def locate(self, anchor_dis, anchor_pos):
        self.anchor_pos = anchor_pos
        (self.x, self.P) = self.ukf.filter_update(self.x, self.Q, anchor_dis)
        return (self.x, self.P)

    def transition_function(self, state, noise):
        return dot(self.A,state)

    def observation_function(self, state, noise):
        return linalg.norm(state[0:3] - self.anchor_pos)

if __name__ == '__main__':

    uwb = UWBLocation()
    
    N = 100

    x = array([[i, i, i*0.4, 4.0/N, 4.0/N, 0.4*4.0/N] for i in linspace(0,4,N)])
    
    anchor = array([[-0.1,-0.1,0.8],[4,0,1.3],[4,4,0.2],[0,4,1.2]])
    
    y = array([[linalg.norm(x[i,0:3]-anchor[i%4])] for i in range(0,N)])

    measure = y + random.randn(N,1)*0.1

    xe = zeros(x.shape)
    p  = zeros((N,x.shape[1],x.shape[1]))

    for i in range(0, N-1):
        xe[i+1], p[i+1] = uwb.locate(measure[i], anchor[i%4])

    t = linspace(0, 10, N) 

    fig = plt.figure()
    ax = fig.add_subplot(121, projection='3d')
    ax.plot(anchor[:,0],anchor[:,1],anchor[:,2],marker='o',linewidth=3)
    ax.plot(xe[:,0], xe[:,1], xe[:,2])
    ax.plot(x[:,0], x[:,1], x[:,2])

    ax = fig.add_subplot(222)
    ax.plot(abs(xe[:,0]-x[:,0]),color = 'red')
    ax.plot(abs(xe[:,1]-x[:,1]),color = 'blue')
    ax.plot(abs(xe[:,2]-x[:,2]),color = 'black')
    
    ax = fig.add_subplot(224)
    ax.plot(abs(xe[:,3]),color = 'red')
    ax.plot(abs(xe[:,4]),color = 'blue')
    ax.plot(abs(xe[:,5]),color = 'black')

    plt.show()


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


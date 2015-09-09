#!/usr/bin/env python
''' The state equation, dx/dt = f(x,t0,u)
    Computes the derivative of state at time t0 on the condition of input u.
    x[0:3] --> Position in ned frame
    x[3:7] --> Quaternion
    x[7:10] --> Velocity in ned frame
    x[10]   --> Bais in Yaw direction of body frame
    
    u[0:3] --> Accelaration in body frame
    u[3:6] --> Angle rate of body frame expressed in inertial frame  '''

from sslib import *

g  = 9.80665   
Q  = zeros((11,11))
Q[ 0:3,  0:3] =  1*eye(3)
Q[ 3:7,  3:7] =  0.001*eye(4)
Q[7:10, 7:10] =  0.0064*eye(3)
Q[10,10]      =  0.00000000001



def state_equation(x, t0, u):
    [p, q, v, b]     = [x[0:3], x[3:7], x[7:10], x[10]]
    [a, wx, wy, wz]  = [u[0:3], u[3], u[4], u[5]]        
    [qx, qy, qz, qw] = [q[0], q[1], q[2], q[3]]  
    #positon transition
    dev_p = v
    #quaternion transition
    R = array([[0,  -wx, -wy, -wz ],
               [wx,   0, -wz,  wy ],
               [wy,  wz,  0 , -wx ],
               [wz, -wy,  wx,  0  ]])
    dev_q = 0.5 * dot(R, q)   
    #veloctiy transition
    T = array([[1-2*qy*qy-2*qz*qz,   2*qx*qy-2*qz*qw,   2*qx*qz+2*qy*qw],
               [2*qx*qy + 2*qz*qw, 1-2*qx*qx-2*qz*qz,   2*qy*qz-2*qx*qw],
               [2*qx*qz-2*qy*qw  , 2*qy*qz + 2*qx*qw, 1-2*qx*qx-2*qy*qy]])
    dev_v = dot(inverse(T), a) + array([0,0,g])   
    #yaw bias transition
    dev_b = 0  
    #merge state transition
    dev_x = hstack((dev_p, dev_q, dev_v, dev_b))
    return dev_x

class UWBLocation:
    def __init__(self):
        self.N = 11
        self.M = 5
        self.x = zeros((1,self.N))[0]
        self.R = zeros((5,5))
        self.R[0,0] = 0.00001
        self.R[1:5,1:5] = eye(4)*0.0001
        self.ukfinit()
        self.state_equation = copy.deepcopy(state_equation)
        self.u = tuple([[0,0,-g,0,0,0]])
              
    def ukfinit(self):
        self.ukf = AdditiveUnscentedKalmanFilter(n_dim_obs = self.M, n_dim_state = self.N,
                                        transition_functions     = self.transition_function,
                                        observation_functions    = self.observation_function,
                                        transition_covariance    = Q,
                                        observation_covariance   = self.R,
                                        initial_state_mean       = self.x,
                                        initial_state_covariance = Q)

    def locate(self, state, state_cov, delt_time, anchor_dis, anchor_pos, quaternion, linear_acc, angular_rate):
        self.anchor_pos = anchor_pos
        self.delt_time  = delt_time 
        self.u = tuple([hstack((linear_acc, angular_rate))])
        (self.x, self.P) = self.ukf.filter_update(state, state_cov, hstack((anchor_dis, quaternion)))
        return (self.x, self.P)

    def transition_function(self, state):
        return odeint(self.state_equation, state, [0, self.delt_time], self.u)[1]

    def observation_function(self, state):
        return hstack((linalg.norm(state[0:3] - self.anchor_pos), state[3:7]))









class FastVisionLocation:
    def __init__(self):
        self.M = 2
        self.N = 3
        self.A = [[1, 0, 0], [0, 1, 0],[0, 0, 1]]
        self.C = [[1, 0, 0], [0, 1, 0]]
        kf = KalmanFilter(n_dim_obs = self.M, n_dim_state = self.N,
                          transition_matrices=self.A, observation_matrices=self.C)

class IMULocation:
    def __init__(self):
        self.N = 10
        self.M = 3
        self.x = zeros((1,self.N))[0]
        self.R = eye(3)*0.02
        self.P = Q
        self.time = -1
        self.ukfinit()
        self.state_equation = copy.deepcopy(state_equation)
        
    def ukfinit(self):

        self.ukf = AdditiveUnscentedKalmanFilter(n_dim_obs = self.M, n_dim_state = self.N,
                                        transition_functions     = self.transition_function,
                                        observation_functions    = self.observation_function,
                                        transition_covariance    = Q,
                                        observation_covariance   = self.R,
                                        initial_state_mean       = self.x,
                                        initial_state_covariance = Q)

    def locate(self, state, state_cov, delt_time, euler_angle, linear_acc, angular_rate):
        self.delt_time = delt_time
        self.u = tuple((hstack((linear_acc, angular_rate))))
        (self.x, self.P) = self.ukf.filter_update(state, state_cov, euler_angle)
        print linalg.norm(euler_angle-self.x[3:6])
        return (self.x, self.P)

    def transition_function(self, state):
        return odeint(self.state_equation, state, [0, self.delt_time], tuple([self.u]))[1]

    def observation_function(self, state):
        return hstack((state[3:6]))
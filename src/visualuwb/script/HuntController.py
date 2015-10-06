#!/usr/bin/env python
from roslib import *
from visualuwb.srv import *

def PointTranslation(b, a):
    x = b.x - a.x
    y = b.y - a.y
    z = b.z - a.z
    return Point(x,y,z)

def VectorLength2D(p):
    return sqrt(p.x*p.x+p.y*p.y)

def cart2pol(x, y):
    rho = sqrt(x**2 + y**2)
    phi = atan2(y, x)
    return(rho, phi)

def pol2cart(rho, phi):
    x = rho * cos(phi)
    y = rho * sin(phi)
    return(x, y)

class HuntController:
    '''The controller need a Pose list poses contains target
        ,a robot and its two neighbors. 
        poses[0]: target
        poses[1]: robot
        poses[2]: neibor1
        poses[3]: neibor2
    '''
    def __init__(self):
        self.k = 1.0 #decision factor
        self.m = 7.0 #factor on surrounding factor
        self.vel_ratio = 0.8
        self.lambd = 4.0/9*pi
    
    def decide(self, poses):
        '''suppose target is on the origin
        '''      
        robot_pos = PointTranslation(poses[1].position, poses[0].position)
        neib1_pos = PointTranslation(poses[2].position, poses[0].position)
        neib2_pos = PointTranslation(poses[3].position, poses[0].position)
        
        #coor[i][0]for i's angle, coor[i][0] for i's distance to target
        coor = []   
        coor.append(cart2pol(robot_pos.x, robot_pos.y))
        coor.append(cart2pol(neib1_pos.x, neib1_pos.y))
        coor.append(cart2pol(neib2_pos.x, neib2_pos.y))

        #get robot and neighbors 's index in coor,fronts are positoin of target
        sortcoor = sorted((e[1], e[0] ,i) for i,e in enumerate(coor))
        robot_index = sortcoor.index((coor[0][1],coor[0][0],0))
        neib_left_index   = (robot_index - 1) % 3
        neib_right_index  = (robot_index + 1) % 3
        
        occupy_angle = 2 * arcsin(self.vel_ratio)
        
        #robot's neigbor angle 
        left_angle  = sortcoor[robot_index][0]  - sortcoor[neib_left_index][0]
        right_angle = sortcoor[neib_right_index][0] - sortcoor[robot_index][0]
        
        #print sortcoor
        #print "neibor angle dis:",left_angle, right_angle
        if left_angle < 0:
            left_angle = left_angle + 2 *pi
        
        if right_angle < 0:
            right_angle = right_angle + 2 *pi
            
        #print left_angle, right_angle
        
        overlap_left = occupy_angle - left_angle
        overlap_right = occupy_angle - right_angle
        
        #print "overlap :",overlap_left, overlap_right
        
        surround_factor = (abs(overlap_left - overlap_right)/(2*pi))**(1/self.m)
        #print "surround factor:",surround_factor
        
        dis_sum = sortcoor[0][1]+sortcoor[1][1]+sortcoor[2][1]
        capture_factor = sin( (sortcoor[robot_index][1]/dis_sum)**(log2(3))*pi)
        
        #print "capture factor", capture_factor
        
        if (overlap_left - overlap_right) < 0 :
            left_or_right = -1   #left
        elif  (overlap_left - overlap_right) > 0:
            left_or_right = 1  #right
        else:
            left_or_right = 0
                
        
        decision_beta = left_or_right * self.lambd *(1 - exp(-self.k * capture_factor * surround_factor))

        #print decision_beta/pi*180
        
        #the target orientaton in terms of robot
        decision_yaw = -decision_beta + atan2(poses[0].position.y - poses[1].position.y, poses[0].position.x - poses[1].position.x)
        if decision_yaw <= -pi:
            decision_yaw = decision_yaw + 2*pi
        elif decision_yaw > pi:
            decision_yaw = decision_yaw - 2*pi
        
        #print decision_yaw/pi*180
        
        q0=poses[1].orientation.w;
        q1=poses[1].orientation.x;
        q2=poses[1].orientation.y;
        q3=poses[1].orientation.z;
        robot_yaw = atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));
        
        #get real turn angle relative to real yaw
        if decision_yaw - robot_yaw >= 0:
            turn_angle = decision_yaw - robot_yaw
            if turn_angle > pi:
                turn_angle = turn_angle - 2*pi
        else:
            turn_angle = decision_yaw - robot_yaw
            if turn_angle <= -pi:
                turn_angle = turn_angle + 2*pi
                  
        #print turn_angle
        #print robot_index,neib_left,neib_right
    
        twist = Twist()
        twist.linear.x = cos(turn_angle)
        twist.linear.y = sin(turn_angle)
        
        
        res = RendezvousResponse()
        res.twist.append(Twist())
        res.twist.append(twist)
        res.twist.append(Twist())
        res.twist.append(Twist())
        return res
    
if __name__ == '__main__':
    hunt = HuntController()
    
    poses = []
        
    pose = Pose() 
    pose.position.x=0
    pose.position.y=0
    pose.orientation.w=1
    poses.append(pose)
    
    pose = Pose() 
    pose.position.x=1
    pose.position.y=1
    pose.orientation.w=1
    poses.append(pose)
    
    pose = Pose() 
    pose.position.x=1
    pose.position.y=0
    pose.orientation.w=1
    poses.append(pose)
    
    pose = Pose() 
    pose.position.x=1.0
    pose.position.y=-1.0
    pose.orientation.w=1
    poses.append(pose)
    
    res = hunt.decide(poses)
    
    print cart2pol(0,-2)
    
    
    
    
    
    
    
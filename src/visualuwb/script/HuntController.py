#!/usr/bin/env python
from sslib import *
from visualuwb.srv import *

class HuntController:
    
    def __init__(self):
        pass
    
    def decide(self, poses):
        res = RendezvousResponse()
        res.twist.append(Twist())
        res.twist.append(Twist())
        res.twist.append(Twist())
        res.twist.append(Twist())
        return res
    
if __name__ == '__main__':
    pass
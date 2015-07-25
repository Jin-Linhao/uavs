#!/usr/bin/env python
import os
ret = os.fork()

print 'res is :', ret

if ret == 0:
    os.system('roslaunch example.launch')

else:
    os.system('rosrun lsd_slam_viewer viewer')
    os.wait()

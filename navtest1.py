#!/usr/bin/env python

import roslib; roslib.load_manifest("nxt_python")
import rospy
import time
import nxt.locator
from nxt.sensor import *
from motor import * #from nxt.motor import *
from sam_navlib import *

"""Feel free to edit this file however you like, it is a good
place to test ideas

http://code.google.com/p/nxt-python/source/browse/trunk/nxt/motor.py?r=20
"""

def navtest1(b):
        rospy.init_node('custom_test')
        
        #initialize the robot
        robot = NXT_Robot(b, PORT_B, PORT_A) #brick, left motor port, right motor port
        
        
        #initialize the sensors. 
        touch = TouchSensor(b, PORT_1)
        sonic = UltrasonicSensor(b, PORT_2)
        light = LightSensor(b, PORT_3)
        
        robot.setPower(100)
        
        robot.forward("3 rotations")
        robot.wait(1)
        robot.pivotTurnLeft("470 degrees")
        robot.wait(1)
        robot.backward("unlimited")
        robot.wait(3)
        robot.brake()
        robot.wait(1)
        robot.curveTurnLeft("4 seconds", 7)
        robot.wait(1)
        robot.pointTurnRight("350 degrees")
        robot.wait(1)
        robot.move_motor("A", "2 rotations", 100)
        robot.wait(1)
        robot.move_motor("B", "3 seconds", 120)

sock = nxt.locator.find_one_brick()
if sock:
	navtest1(sock.connect())
	sock.close()
else:
	print 'No NXT bricks found'

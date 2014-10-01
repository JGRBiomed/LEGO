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

def navtest2(b):
        rospy.init_node('custom_test')
        
        #initialize the robot
        robot = NXT_Robot(b, PORT_B, PORT_A) #brick, left motor port, right motor port
        
        #initialize the sensors. 
        touch = TouchSensor(b, PORT_1)
        sonic = UltrasonicSensor(b, PORT_2)
        light = LightSensor(b, PORT_3)
        
        robot.setPower(100)
        #pointTurnTest(robot)
        #pivotTurnTest(robot)
        #curveTurnTest(robot)
        motorAccessorMutatorTest(b, robot)
        #sensorTest(robot, touch, sonic, light)
        
def pointTurnTest(robot):
        
        print "testing left turns"

        print "point turn degrees"
        robot.pointTurnLeft("360 degrees", True)
        robot.wait(1)
        
        print "point turn rotations"
        robot.pointTurnLeft("2 rotations")
        robot.wait(1)
        
        print "point turn seconds"
        robot.pointTurnLeft("1 second")
        robot.wait(1)
        
        print "point turn unlimited"
        robot.pointTurnLeft("unlimited")
        robot.wait(1)
        robot.brake()
        
        print "testing right turns"
        
        print "point turn degrees"
        robot.pointTurnRight("360 degrees", True)
        robot.wait(1)
        
        print "point turn rotations"
        robot.pointTurnRight("2 rotations")
        robot.wait(1)
        
        print "point turn seconds"
        robot.pointTurnRight("1 second")
        robot.wait(1)
        
        print "point turn unlimited"
        robot.pointTurnRight("unlimited")
        robot.wait(1)
        robot.brake()
        
def curveTurnTest(robot):

        diffFactor = 1
        
        while diffFactor <= 7:
            print "testing left turns"
            
            print "rotations"
            robot.curveTurnLeft("2 rotations", diffFactor)
            robot.wait(1)
            
            print "seconds"
            robot.curveTurnLeft("1 second", diffFactor)
            robot.wait(1)
            
            print "degrees"
            robot.curveTurnLeft("400 degrees", diffFactor)
            robot.wait(1)
            
            print "unlimited"
            robot.curveTurnLeft("unlimited", diffFactor)
            robot.wait(1)
            robot.brake()
            
            robot.wait(1)
            diffFactor = diffFactor + 1
            print "current curve turn factor: " + str(diffFactor)
        
        diffFactor = 1
        while diffFactor <= 7:
            print "testing right turns"
            
            print "rotations"
            robot.curveTurnRight("2 rotations", diffFactor)
            robot.wait(1)
            
            print "seconds"
            robot.curveTurnRight("1 second", diffFactor)
            robot.wait(1)
            
            print "degrees"
            robot.curveTurnRight("400 degrees", diffFactor)
            robot.wait(1)
            
            print "unlimited"
            robot.curveTurnRight("unlimited", diffFactor)
            robot.wait(1)
            robot.brake()
            
            robot.wait(1)
            diffFactor = diffFactor + 1
            print "current curve turn factor: " + str(diffFactor)
        
def pivotTurnTest(robot):
        print "testing left turns"

        print "pivot turn degrees"
        robot.pivotTurnLeft("360 degrees")
        robot.wait(1)
        
        print "pivot turn rotations"
        robot.pivotTurnLeft("2 rotations")
        robot.wait(1)
        
        print "pivot turn seconds"
        robot.pivotTurnLeft("1 second")
        robot.wait(1)
        
        print "pivot turn unlimited"
        robot.pivotTurnLeft("unlimited")
        robot.wait(1)
        robot.brake()
        
        print "testing right turns"
        
        print "pivot turn degrees"
        robot.pivotTurnRight("360 degrees")
        robot.wait(1)
        
        print "pivot turn rotations"
        robot.pivotTurnRight("2 rotations")
        robot.wait(1)
        
        print "pivot turn seconds"
        robot.pivotTurnRight("1 second")
        robot.wait(1)
        
        print "pivot turn unlimited"
        robot.pivotTurnRight("unlimited")
        robot.wait(1)
        robot.brake()
        
def motorAccessorMutatorTest(b, robot):
        print "querying current motor objects"
        robot.getLeftMotor().ident = "old motor 1"
        robot.getRightMotor().ident = "old motor 2"
        print robot.getLeftMotor().ident
        print robot.getRightMotor().ident
        print "assigning new motor objects"
        newmotor1 = Motor(b, PORT_A)
        newmotor2 = Motor(b, PORT_B)
        newmotor2.ident = "new motor 2"
        newmotor1.ident = "new motor 1"
        robot.setLeftMotor(newmotor1)
        robot.setRightMotor(newmotor2)
        print robot.getLeftMotor().ident
        print robot.getRightMotor().ident
        print "setting drive motors"
        robot.setDriveMotors(newmotor1, newmotor2)
        
        print "adding third motor"
        newmotor3 = Motor(b, PORT_C)
        newmotor3.ident = "new motor 3"
        
        robot.setThirdMotor(newmotor3)

        motorlist = robot.getMotorList()
        for thing in motorlist:
            print thing.ident
        
        robot.setDriveMotors(newmotor2, newmotor1)
        robot.motorCheck()
        
        robot.setThirdMotor(None)
        robot.motorCheck()
        
        """        
        Call get_sample() on a sensor object to get its output.
        The touch sensor returns true or false
        The sonic sensor returns a value between 0 and 255
        The light sensor seems to return a number between 100 and 700
        """
        
def sensorTest(robot, touch, sonic, light):      
        #Move forward until the touch sensor is bumped
        robot.forward("unlimited")
        
        #This simulates the "bump" setting
        print "bump the sensor"
        while touch.get_sample() != 1:
            time.sleep(0.1)
        print "release the sensor"
        while touch.get_sample() == 1:
            time.sleep(0.1)
        robot.brake()
        
        #When the touch sensor is pushed, move motor A
        print "push the sensor to continue"
        #This simulates the "pushed" setting
        while touch.get_sample() != 1:
            pass
        print "setting power"
        robot.setPower(100)
        
        print " move motor "
        robot.move_motor("A", "3 rotations", 100)
        
        #When the ultrasonic sensor detects an object is close, pivot turn left
        while sonic.get_sample() > 150:
            print "Ultrasonic Sensor trying to detect proximity: " + str(sonic.get_sample()) + '\r',
            time.sleep(0.1)
        print "\nUltrasonic Sensor detecting proximity: " + str(sonic.get_sample())
        
        print " pivot left "
        robot.pivotTurnLeft("2 seconds")

        #When the light sensor detects darkness, pivot turn right        
        print light.get_sample()
        while light.get_sample() < 400:
            print "Light Sensor trying to detect darkness: " + str(light.get_sample()) + '\r',
            time.sleep(0.1)
        print "\nLight Sensor detected darkness: " + str(light.get_sample())
        
        print " pivot right "
        robot.pivotTurnRight("1222 degrees")
        
        print "push the touch sensor to curve turn left"
        #When the touch sensor is pushed, curve turn left
        while touch.get_sample() != 1:
            pass
        robot.curveTurnLeft("3 rotations", 7)
    

sock = nxt.locator.find_one_brick()
if sock:
	navtest2(sock.connect())
	sock.close()
else:
	print 'No NXT bricks found'

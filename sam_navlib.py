#!/usr/bin/env python

import roslib; roslib.load_manifest("nxt_python")
import rospy
import time
import nxt.locator
from nxt.sensor import *
from motor import * #from nxt.motor import *

"""This class models an NXT robot and allows you to send higher level navigational commands to it

http://code.google.com/p/nxt-python/source/browse/trunk/nxt/motor.py?r=20 appears to link to the code that you import with nxt.motor

http://code.google.com/p/nxt-python/source/browse/trunk/nxt/motor.py?r=20\ links to a more updated version under the GNU GPL
"""

class NXT_Robot:

    def __init__(self, b, leftport, rightport, thirdport = None):
        self.leftmotor = Motor(b, leftport)
        self.rightmotor = Motor(b, rightport)
        self.power = 100
        self.executing = False
        self.motors = SynchronizedMotors(self.leftmotor, self.rightmotor, 0)
        if thirdport != None:
            self.thirdmotor = Motor(b, thirdport)
        else:
            self.thirdmotor = None

#Accessors and Mutators for the robot's motors        
    def getLeftMotor(self):
        return self.leftmotor
        
    def getRightMotor(self):
        return self.rightmotor
        
    def setLeftMotor(self, l):
        self.leftmotor = l
        self.motorCheck()
        
    def setRightMotor(self, r):
        self.rightmotor = r
        self.motorCheck()
        
    def setDriveMotors(self, l, r):
        self.leftmotor = l
        self.rightmotor = r
        self.motors = SynchronizedMotors(self.leftmotor, self.rightmotor, 0)
        self.motorCheck()
        
    def setThirdMotor(self, t):
        if self.thirdmotor == None:
            self.thirdmotor = t
        else:
            self.thirdmotor = t
        self.motorCheck()
        
    def getThirdMotor(self):
        if self.thirdmotor != None:
            return self.thirdmotor
        else:
            raise UnboundLocalError("There is no motor specified as the third motor!") 
            
    def motorCheck(self):
        print "Left drive motor: " + self.findPort(self.leftmotor)
        print "Right drive motor: " + self.findPort(self.rightmotor)
        print "Third motor: " + self.findPort(self.thirdmotor)
        
    def findPort(self, motor):
        if motor == None:
            return "No motor assigned"
        elif motor.port == PORT_A:
            return "Port A"
        elif motor.port == PORT_B:
            return "Port B"
        elif motor.port == PORT_C:
            return "Port C"
        else:
            raise ValueError("Invalid Port")
            
    def getMotorList(self):
        motorlist = []
        if self.leftmotor != None:
            motorlist.append(self.leftmotor)
        if self.rightmotor != None:
            motorlist.append(self.rightmotor)
        if self.thirdmotor != None:
            motorlist.append(self.thirdmotor)
        return motorlist
        
#The power determines the speed and direction of the motor.
#This should be an integer between -127 and 127
    def setPower(self, power):
        if power < -127 or power > 127:
            raise ValueError("Invalid Power. Please enter an integer between -127 and 127")
        else:
            self.power = power
            
    def getPower(self):
        return self.power

    """        
    This method sends a command to both motors, moving the robot forward.
    It expects an input of the form "x movement_type"
    x is the number of units to move
    movement_type specifies the unit: seconds, rotations, or degrees.
    Example inputs:
        forward("unlimited")
        forward("3 rotations")
        forward("423 degrees")
        forward("3 seconds") 
    """

    def forward(self, string):
        if string == "unlimited":
            self.motors.run()
        else:
            instruction = string.split()
            amount = float(instruction[0])
            unit = instruction[1]
            if unit == "seconds" or unit == "second":
                self.move_seconds(amount),
            elif unit == "rotations" or unit == "rotation": 
                self.move_rotations(amount),
            elif unit == "degrees" : 
                self.move_degrees(amount)
            else:
                raise ValueError("Cannot parse the unit type\n" + "I do not understand what unit " + unit + " is.\n" +  "Units can only be either: unlimited, rotations, seconds, or degrees ")

#Backward flips the power and calls forward            
    def backward(self, string):
        self.power = self.power * -1
        self.forward(string)
        self.power = self.power * -1
        
#A thin wrapper around the SynchronizedMotors brake method    
    def brake(self):
        self.motors.brake()
            
#I felt these subroutines might be useful in the future, so I made them their own methods
    def move_rotations(self, rotations):
        degrees = rotations * 360
        self.motors.turn(self.power, degrees)
        
    def move_degrees(self, degrees):
        self.motors.turn(self.power, degrees)
        
    def move_seconds(self, seconds):
        self.motors.run(self.power)
        start = rospy.Time.now()
        while rospy.Time.now() < start + rospy.Duration(seconds):
            pass
        self.motors.brake()
        
    """This is similar to the forward and backwards methods, but they apply only to a single motor.

    When called directly, it accepts "A", "B", and "C" as parameters, specifying the motor being moved.
    """
    def move_motor(self, motor, command_string, power):
        motorlist = self.getMotorList()
        if motor == "A":
            for nxtmotor in motorlist:
                if nxtmotor.port == PORT_A:
                    motor = nxtmotor
            motor = self.leftmotor
        elif motor == "B":
            for nxtmotor in motorlist:
                if nxtmotor.port == PORT_B:
                    motor = nxtmotor
        elif motor == "C":
            for nxtmotor in motorlist:
                if nxtmotor.port == PORT_A:
                    motor = nxtmotor
        elif type(motor) is Motor:
            pass
            
        else:
            raise ValueError(str(motor) + " is not a valid motor port or motor. Please enter A, B, or C, or pass in a motor object.")
        if power < -127 or power > 127:
            raise ValueError("Invalid Power. Please enter an integer between -127 and 127")
        else:
            self.power = power
            
        if command_string == "unlimited":
            motor.run()
            
        else:
            instruction = command_string.split()
            amount = float(instruction[0])
            unit = instruction[1]
            
            if unit == "seconds" or unit == "second":
                motor.run()
                start = rospy.Time.now()
                while rospy.Time.now() < start + rospy.Duration(amount):
                    pass
                motor.brake()
            elif unit == "rotations" or unit == "rotation": 
                motor.turn(self.power, amount*360),
            elif unit == "degrees" : 
                motor.turn(self.power, amount)
            else:
                raise ValueError("Cannot parse the unit type\n" + "I do not understand what unit " + unit + " is.\n" +  "Units can only be either: unlimited, rotations, seconds, or degrees ")
                
#A standard turn where one motor turns on for a set duration
    def pivotTurnLeft(self, string):
        self.move_motor(self.rightmotor, string, self.power)
        
    def pivotTurnRight(self, string):
        self.move_motor(self.leftmotor, string, self.power)
        
#One wheel moves forward, one wheel moves backward
    def pointTurnRight(self, string, debug = False):
       
        if string == "unlimited":
            if debug == True:
                print "command string is unlimited"
        
            self.leftmotor.run(self.power)
            self.rightmotor.run(-1 * self.power)
        else:
        
            instruction = string.split()
            amount = float(instruction[0])
            unit = instruction[1]
            if unit == "seconds" or unit == "second":
                if debug == True:
                    print "command string is " + str(amount) + " seconds"
                self.pointTurnSeconds(self.leftmotor, self.rightmotor, amount)
            elif unit == "rotations" or unit == "rotation": 
                if debug == True:
                    print "command string is " + str(amount) + " rotations"
                self.pointTurnRotations(self.leftmotor, self.rightmotor, amount)
            elif unit == "degrees" : 
                if debug == True:
                    print "command string is " + str(amount) + " degrees"
                self.pointTurnDegrees(self.leftmotor, self.rightmotor, amount)
            else:
                raise ValueError("Cannot parse the unit type\n" + "I do not understand what unit " + unit + " is.\n" +  "Units can only be either: unlimited, rotations, seconds, or degrees ")
        
    def pointTurnLeft(self, string, debug = False):
        
               
        if string == "unlimited":
            if debug == True:
                print "command string is unlimited"
        
            self.leftmotor.run(-1 *self.power)
            self.rightmotor.run(self.power)
        else:
        
            instruction = string.split()
            amount = float(instruction[0])
            unit = instruction[1]
            if unit == "seconds" or unit == "second":
                if debug == True:
                    print "command string is " + str(amount) + " seconds"
                self.pointTurnSeconds(self.rightmotor, self.leftmotor, amount)
            elif unit == "rotations" or unit == "rotation": 
                if debug == True:
                    print "command string is " + str(amount) + " rotations"
                self.pointTurnRotations(self.rightmotor, self.leftmotor, amount)
            elif unit == "degrees" : 
                if debug == True:
                    print "command string is " + str(amount) + " degrees"
                self.pointTurnDegrees(self.rightmotor, self.leftmotor, amount)
            else:
                raise ValueError("Cannot parse the unit type\n" + "I do not understand what unit " + unit + " is.\n" +  "Units can only be either: unlimited, rotations, seconds, or degrees ")
        
    #The first motor moves forward, the second backward
    def pointTurnSeconds(self, first_motor, second_motor, seconds):
    
        first_motor.run(self.power)
        second_motor.run(-1 * self.power)
        start = rospy.Time.now()
        while rospy.Time.now() < start + rospy.Duration(seconds):
            pass
        self.motors.brake()
        
    def pointTurnDegrees(self, first_motor, second_motor, degrees):
    
        goaltachfirst = first_motor.get_tacho().tacho_count + degrees
        goaltachsecond = second_motor.get_tacho().tacho_count - degrees
        leftreached = False
        rightreached = False
        first_motor.run(self.power)
        second_motor.run(-1 * self.power)
        while leftreached == False or rightreached == False:
            
            if first_motor.get_tacho().tacho_count > goaltachfirst:
                first_motor.brake()
                leftreached = True
            if second_motor.get_tacho().tacho_count < goaltachsecond:
                second_motor.brake()
                rightreached = True
        
        
    def pointTurnRotations(self, first_motor, second_motor, rotations):
        degrees = int(rotations * 360)
        self.pointTurnDegrees(first_motor, second_motor, degrees)
        
#Both motors are on, with one moving faster than the other
    def curveTurnLeft(self, amount, degree_of_turn):
        self.motors = SynchronizedMotors(self.leftmotor, self.rightmotor, degree_of_turn)
        self.forward(amount)
        self.motors = SynchronizedMotors(self.leftmotor, self.rightmotor, 0)
        
    def curveTurnRight(self, amount, degree_of_turn):
        self.motors = SynchronizedMotors(self.rightmotor, self.leftmotor, degree_of_turn)
        self.forward(amount)
        self.motors = SynchronizedMotors(self.leftmotor, self.rightmotor, 0)
        
#This simulates a wait block
    def wait(self, seconds):
        start = rospy.Time.now()
        while rospy.Time.now() < start + rospy.Duration(seconds):
            pass
        return
    
#This tracks a line for a predefined number of seconds
    def trackLine(self, lightsensor, seconds, threshhold):
        
        print "Tracking!"
        start = rospy.Time.now()
        while rospy.Time.now() < start + rospy.Duration(seconds):
            if lightsensor.get_sample() < threshhold:
                self.rightmotor.brake()
                self.leftmotor.run(80)
            else:
                self.leftmotor.brake()
                self.rightmotor.run(80)
        self.motors.brake()
        print "Tracking complete"
        return

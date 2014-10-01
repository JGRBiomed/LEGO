This document informs the user of the various methods available to them in the sam_navlib.py file. 

The purpose of this file is to provide a wrapper that allows you to issue high level commands when controlling an NXT Mindstorms robot using python with ROS.

This file relies on a lower level wrapper/interface provided by the file motor.py, code that was found at 

http://code.google.com/p/nxt-python/source/browse/trunk/nxt/motor.py?r=20\

============================================================
The Main Idea:

    sam_navlib.py allows you to construct an NXT_Robot object and issue it high level 
    commands.
    
    You can type simple and easily readable code like
    
        robot = NXT_Robot(b, PORT_A, PORT_B)        #Constructor for a 2 motor robot object
        
        robot.setPower(120)                         #Set the speed, from -127 to 127
        robot.forward("3 rotations")                #Both motors turn on for 3 rotations
        robot.pivotTurnLeft("470 degrees")          #Right motor turns on for 470 degrees
        robot.backward("unlimited")                 #Both motors turn on for 3 seconds
        robot.wait(3)                               
        robot.brake()                               
        robot.curveTurnLeft("4 seconds", 7)         #Motors turn with a speed differential of factor 7
        robot.pointTurnRight("350 degrees")         #Left motor moves forward, right motor backwards
        robot.move_motor("A", "2 rotations", 100)   #Direct commands to the motor in the specified port
        robot.move_motor("B", "3 seconds", 120)
        
    The NXT_Robot constructor takes the brick and ports of the drive motors as parameters.
    The first port provided should be connected to the motor on the left.
    
    The constructor is overloaded - you can also give it a brick and three motor ports.
    The first two ports should be drive motors, the third port the arm motor.
    Just as before, the first motor port should be connected to the motor on the left.
    
        Example: robot = NXT_Robot(b, PORT_A, PORT_B, PORT_C)
    
    The forward, backward, and curveTurn methods let you control both drive motors at once
    The pivotTurnLeft method turns on the motor on the right for the specified duration
    The pivotTurnRight method turns on the motor on the left for the specified duration
    
    The move_motor method gives you fine grained control similar to robot C
    It takes as parameters a motor, a command_string, and a power.

============================================================
:: Writing a Python File ::

""" BEGINNING OF STENCIL

Create a file called python_controlled_robot.py in the same folder that contains both
sam_navlib.py and motor.py

Copy and paste this stencil into that file.
After you have filled in the stencil, you have to make it executable.

Navigate to the folder containing python_controlled_robot.py and type in the command line:

chmod +x python_controlled_robot.py

To run your program, from the folder containing the file, type into the command line:

./python_controlled_robot.py
"""

#!/usr/bin/env python

import roslib; roslib.load_manifest("nxt_python")
import rospy
import time
import nxt.locator
from nxt.sensor import *
from motor import *
from sam_navlib import *

def python_controlled_robot(b):
        rospy.init_node('custom_test')
        
        # Initialize the robot and any sensors here.
        # Give it commands to execute!


sock = nxt.locator.find_one_brick()
if sock:
	python_controlled_robot(sock.connect())
	sock.close()
else:
	print 'No NXT bricks found'
	
"""END OF STENCIL """

============================================================
NAVIGATION WITH SENSORS:
    
    To have a robot object rely on sensors, we must initialize a robot object and the sensor objects.
    Sensor objects are initialized by giving the relevant constructor a reference to the brick, and to the port the sensor is connected to.
    
    Example:
    
        touch = TouchSensor(b, PORT_1)
        sonic = UltrasonicSensor(b, PORT_2)
        light = LightSensor(b, PORT_3)
        
        Call get_sample() on a sensor object to get its output.
        The touch sensor returns true or false
        The sonic sensor returns a value between 0 and 255
        The light sensor seems to return a number between 100 and 700
        
-----------------------------------------------------------
:: Wait for Time ::

    robot.wait(int)
    
~~~~~~~~~~~~~~~~~~~
:: Touch Sensor ::
    
Wait for touch (pressed):
    
    while touch.get_sample() != 1:
        pass 
        
Wait for touch (released):

    while touch.getsample() != 0:
        pass
        
Wait for touch (bumped):

    while touch.get_sample() != 1:
        pass
    while touch.get_sample() != 0:
        pass
        
~~~~~~~~~~~~~~~~~~~

:: Wait for Ultrasonic Sensor ::

    while sonic.get_sample() < 125:
        pass
        
    OR
    
    while sonic.get_sample() > 125:
        pass
        
    Here 125 is an arbitrary threshhold - choose one appropriate for the task
        
~~~~~~~~~~~~~~~~~~~

:: Wait for Light Sensor ::

    while light.get_sample() > 400:
        pass
        
    OR
   
    while light.get_sample() < 400:
        pass
        
    Here 400 is an arbitrary threshhold - choose one appropriate for the task
~~~~~~~~~~~~~~~~~~~

:: Specify a sampling rate ::

If you prefer to specify a sampling rate, instead of pass type

    time.sleep(delay_between_samples)
    
    where delay_between_samples is a unit in seconds, e.g.
    
    while touch.get_sample() != 1:
        time.sleep(0.1) 
        
    This code will check the sensor, wait one/tenth of a second, then check it again.
    You can specify your own delay.


~~~~~~~~~~~~~~~~~~~

:: Track a Line ::

There is a built in function that runs a preprogrammed line following routine.

    robot.trackLine(lightsensor, seconds_to_track, threshhold)
    
It takes as parameters a light sensor object, the number of seconds to track the line, and the threshhold that allows it to distinguish between black and white. A number lower than the threshhold will be considered black, a number higher than the threshhold will be considered white.
        
        
If you would like to write a custom line following program, you will have to use the built in ros timer functions.

Sample code would look like this:

 def trackLine(self, lightsensor, seconds, threshhold):
    
        start = rospy.Time.now()
        while rospy.Time.now() < start + rospy.Duration(seconds):
            if lightsensor.get_sample() < threshhold:
                self.rightmotor.brake()
                self.leftmotor.run(80)
            else:
                self.leftmotor.brake()
                self.rightmotor.run(80)
        self.motors.brake()
        return
============================================================

CONSTRUCTOR

NXT_Robot(brick, left motor port, right motor port)
    
    OR

NXT_Robot(brick, left motor port, right motor port, third motor port)

-------------------------------------------------------------

METHODS

:: Motor Accessors ::

getLeftMotor()

    returns the motor object assigned as the left motor

getRightMotor() 
    
    returns the motor object assigned as the right motor

getThirdMotor()
    
    returns the motor object assigned as the third motor

~~~~~~~~~~~~~~~~~~~~~~~~~

:: Motor Mutators ::

setLeftMotor(motor) 
    
    assigns the motor passed in as the left motor
    
setRightMotor(motor)

    assigns the motor passed in as the right motor
    
setDriveMotors(leftmotor, rightmotor)

    assigns the first motor passed in as the left motor, and the second as the right motor

~~~~~~~~~~~~~~~~~~~~~~~~~

:: Checking Motor Assignment ::

motorCheck() 
   
    prints to standard out strings containing the motors the robot thinks is connected
    
findPort(motor)

    returns a string indicating what port a motor object is connected to 
    
getMotorList()

    returns a list of all the motor objects the robot knows about

~~~~~~~~~~~~~~~~~~~~~~~~~

:: Power ::

getPower()

    returns the current power

setPower(int)
    
    Takes in an integer between -127 and 127. 
    Negative numbers move backward, positive move forward.
    The greater |int|, the faster it goes. 
    Almost all movement methods refer to this value to determine how fast to move.

~~~~~~~~~~~~~~~~~~~~~~~~~

:: Movement Methods ::

The following movement methods take in a command_string as a parameter.

This command_string can take one of four forms::
    command_command_string == "unlimited"
    command_command_string == "int rotations"
    command_command_string == "int degrees"
    command_command_string == "int seconds"
    
Where int is the integer number of units to move.

------------------------

forward(command_string)
backward(command_string)
    
    Examples:
        forward("unlimited")
        backward("3 rotations")
        forward("497 degrees")
        backward("5 seconds")
        
    Note: Avoid asking for fewer than 50 degrees of movement. 
    Due to the limitations of motors.py you may get erratic behavior.

------------------------

wait(int)

    The robot will take the next action in int seconds.

------------------------
    
brake()  

    tells the motors to stop moving

------------------------

move_motor(motor, command_string, power)
    
    motor is a parameter that can be either a motor object or "A", "B", or "C"
        
    Examples:
        move_motor("A", "3 rotations", 100)
        move_motor("B", "unlimited", 120)
        move_motor(motor_object, "340 degrees", 80)
        
    Note: Avoid asking for fewer than 50 degrees of movement. 
    Due to the limitations of motors.py you may get erratic behavior.
    
------------------------

curveTurnLeft(command_string, magnitude of turn)
curveTurnRight(command_string, magnitude of turn)

    The robot moves forward in a curved turn. 
    The larger the magnitude of the turn, the smaller the turning radius.
    
------------------------

pointTurnLeft(command_string)
pointTurnRight(command_string)

    One wheel of the robot moves forward, the other backward.
    
    It parses the command string and sends the appropriate parameters to the following methods,
    which can be directly called if finer control is desired:
    
pointTurnRotations(first_motor, second_motor, rotations)
pointTurnDegrees(first_motor, second_motor, degrees)
pointTurnSeconds(first_motor, second_motor, degrees)

    The motor passed in as first_motor moves forward
    The motor passed in as second_motor moves backward
        

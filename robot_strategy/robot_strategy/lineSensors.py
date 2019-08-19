#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt8
import numpy as np
from enum import Enum

class Sides(Enum):
    LEFT = 0
    RIGHT = 1
    FRONT = 2
    BACK = 3

    def __int__(self):
        return self.value

class LineSensor(object):
    """
    This class is an abstraction of a group of sensors to line detection. I implemented a function to keep track of a line with those sensors

    In our robot, we have 1 pololu's sensor - with 2 phototransitorseach - in each of the following: Left, Front, Right and Back of the robot.

    The output of the readLine() is an array with the line position for each pair of sensor:
        -2 : line is in front/right of the robot, and all the sensors are out of the line
        -1 : line is in front/right of the robot, and the front sensor is over the line
        0 : the robot is over the line
        1 : contrary of -1
        2 : contrty of -2
    
    Subscribe:
        /pololuSensor: UInt8
            each bit of this 8bits int is a sensor reading True (1) or False (0)
    
    """
    def __init__(self):

        self._subscribers = rospy.Subscriber( "/pololuSensor", UInt8, self.__callback)
        self._error = np.ones(4) * - 2

    def __callback(self, msg):
        """
        Callback to the ROS subscriber

        Parameters
        ----------
        msg : std_msgs/UInt8
            The readings of the sensor
        
        """
        # Check if at least one sensor is over the line 
        sensorValueA = np.array( [ (msg.data & 1) != 0, (msg.data & 4) != 0, (msg.data & 16) != 0, (msg.data & 64) != 0])
        sensorValueB = np.array([ (msg.data & 2) != 0, (msg.data & 8) != 0, (msg.data & 32) != 0, (msg.data & 128) != 0])
        onLine =  sensorValueA + sensorValueB
        
        for i in range(4):
            if (onLine[i]):
                self._error[i] = 1*sensorValueA[i]-sensorValueB[i]
            else:
                self._error[i] = 2 if self._error[i] > 0 else - 2

    def reset(self, resetToMinimun=True):
        """
        Reset the tracking of the line. This is useful to start alinging with a new line

        Parameters
        ----------
        resetToMinimun : boolean (default True)
            if True, the new values are 0, otherwise they are the ( numberOfSensors - 1 ) * 1000
        """

        for i in range(len(self._error)):
            if resetToMinimun:
                self._error = np.ones(4)*-2
            else:
                self._error = np.ones(4)*2

    def readLines(self):
      
        return self._error
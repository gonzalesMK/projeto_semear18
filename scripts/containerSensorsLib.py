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

class ContainerSensors(object):
    """
    This class is an abstraction of a group of sensors to line detection. I implemented a function to keep track of a line with those sensors

    In our robot, we have 1 pololu's sensor - with 2 phototransitorseach - in each of the following: Left, Front, Right and Back of the robot.

    This class should subscribe a topics that advertise UInt8 -each bit is a sensor reading

    The output of the readLine() is an array with the line position for each pair of sensor:
        -2 : line is in front/right of the robot, and all the sensors are out of the line
        -1 : line is in front/right of the robot, and the front sensor is over the line
        0 : the robot is over the line
        1 : contrary of -1
        2 : contrty of -2
    
    """
    def __init__(self):

        self._subscribers = rospy.Subscriber( "/containerSensors", UInt8, self.__callback)

    def __callback(self, msg):
        """
        Callback to the ROS subscriber

        Parameters
        ----------
        msg : std_msgs/UInt8
            The readings of the sensor
        
        """
        # Check if at least one sensor is over the line 
        self.sensor = [ (msg.data & 1) , (msg.data & 2) ]

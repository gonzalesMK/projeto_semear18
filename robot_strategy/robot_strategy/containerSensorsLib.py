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
    """
    def __init__(self):

        self._subscribers = rospy.Subscriber( "/containerSensor", UInt8, self.__callback)
        self.sensor = [0,0]

    def __callback(self, msg):
        """
        Callback to the ROS subscriber

        Parameters
        ----------
        msg : std_msgs/UInt8
            The readings of the sensor
        
        """
        # Check if at least one sensor is over the line 
        self.sensor = [ not (msg.data & 1) , not (msg.data & 2) ]

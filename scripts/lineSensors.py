#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt16
from functools import partial
import numpy as np


class LineSensor:
    """
    This class is an abstraction of a group of sensors to line detection. I implemented a function to keep track of a line with those sensors

    In our robot, we have 1 pololu's sensor - with 2 phototransitorseach - in each of the following: Left, Front, Right and Back of the robot.

    This class should subscribe to each sensor topic, and the topics should advertise AnalogValue UInt16 with the respective readings

    Parameters
    ----------
    topicsToSubscribe : list of Strings
        Each string is a name of a topic to subscribe to - THE TOPICS SHOULD BE IN THE RIGHT ORDER TO BE USED
    
    """

    def __init__(self, topicsToSubscribeTo):

        if( topicsToSubscribeTo is not list ):
            print("The given topics names are not a list")
            return -1 
        
        self.sensorsTopics = topicsToSubscribeTo
        self._BLACK_LOW_LIMIT= 800 # more than that is considered black
        self.numberOfSensors = len(topicsToSubscribeTo)
        self._maxValue = (self.numberOfSensors - 1) * 1000
        self.values = list(0 for topic in topicsToSubscribeTo)
        self._subscribers = []
        self.__linePose = 0 
        for i in range(self.numberOfSensors):
            self._subscribers.append( rospy.Subscriber(topicsToSubscribeTo[i], partial(self.__callback, sensorNumber = i)))


    def __callback(self, msg, sensorNumber):
        """
        Callback to the ROS subscriber

        Parameters
        ----------
        msg : std_msgs/UInt16
            The readings of the sensor
        
        sensorNumber: int
            The number of the sensor to set the value to
        """
        
        self.values[sensorNumber] = msg.data


    def reset(self, resetToMinimun=True):
        """
        Reset the tracking of the line. 

        Parameters
        ----------
        resetToMinimun : boolean (default True)
            if True, the new values are 0, otherwise they are the ( numberOfSensors - 1 ) * 1000
        """

        for i in range(len(self.values)):
            if resetToMinimun :
                self.values = 0
            else:
                self.values = (self.numberOfSensors - 1) * 1000

    def readLine(self):
      
        # Check if at least one sensor is over the line 
        onLine = np.sum( self.values > self.BLACK_LOW_LIMIT) >= 1 

        avg = np.sum( self.values[i] * (i * 1000) for i in range(len(self.values))) # We are not using the first value of the array ?! Maybe change that later on
        sum = np.sum( self.values )

        if sum == 0:
            avg = 0
        else:
            avg /= sum

        
        if (onLine) : # If the line has been detected
            self.__linePose = avg
        else:
            self.__linePose = 0 if avg < (self.maxValue/2) else self._maxValue
        
        return self.__linePose
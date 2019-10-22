#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt8, Bool
import numpy as np
from enum import Enum
from enum import IntEnum
from functools import partial

"""
Original: 
FRONT = 0
BACK = 1
LEFT = 2
RIGHT = 3

Novo

"""
class Sides(Enum):
    FRONT =  0  
    BACK = 1
    LEFT =  2
    RIGHT = 3

    def __int__(self):
        return self.value

class sensorSides(IntEnum):
    FL = 0
    FR = 1
    BL = 2
    BR = 3
    LF = 4
    LB = 5
    RF = 6
    RB = 7


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

        self._error = np.ones(4, dtype=float) * - 2
        self._readings = np.ones(8, dtype=float) * 20
        self._max_readings = np.ones(8, dtype=float) * 100
        self._min_readings = np.ones(8, dtype=float) * 20

        rospy.Subscriber( "/pololuSensor", UInt8, self.__callback)
        rospy.Subscriber( "/pololuSensorFL", UInt8, partial( self.__callback_sensor, sensorSides.FL))
        rospy.Subscriber( "/pololuSensorFR", UInt8, partial( self.__callback_sensor, sensorSides.FR))
        rospy.Subscriber( "/pololuSensorBL", UInt8, partial( self.__callback_sensor, sensorSides.BL))
        rospy.Subscriber( "/pololuSensorBR", UInt8, partial( self.__callback_sensor, sensorSides.BR))
        rospy.Subscriber( "/pololuSensorLF", UInt8, partial( self.__callback_sensor, sensorSides.LF))
        rospy.Subscriber( "/pololuSensorLB", UInt8, partial( self.__callback_sensor, sensorSides.LB))
        rospy.Subscriber( "/pololuSensorRF", UInt8, partial( self.__callback_sensor, sensorSides.RF))
        rospy.Subscriber( "/pololuSensorRB", UInt8, partial( self.__callback_sensor, sensorSides.RB))
        
        rospy.Subscriber( "/turnOnPololuSensors", Bool, self.__enableCb)
        self._is_on = False
        self.pub_enable = rospy.Publisher('/turnOnPololuSensors', Bool, queue_size=10)  
       
        print("Turning on sensors")
        self.pub_enable.publish(True)  
        
        while( not self._is_on and not rospy.is_shutdown() ):
            print("Trying again")
            self.pub_enable.publish(True)  
            rospy.Rate(20).sleep()

        
    def __callback_sensor(self, sensorNumber, msg):
        
        
        if self._max_readings[sensorNumber] < msg.data:
            self._max_readings[sensorNumber] = msg.data

        if self._min_readings[sensorNumber] > msg.data  :
            self._min_readings[sensorNumber] = msg.data

        self._readings[sensorNumber] =  (msg.data - self._min_readings[sensorNumber] ) / ( self._max_readings[sensorNumber] - self._min_readings[sensorNumber] )  * 100
        
    def __enableCb(self,msg):
        """
        Callback to the ROS subscriber. This one is to be sure that the enable was published

        Parameters
        ----------
        msg : std_msgs/UInt8
            The readings of the sensor
        
        """

        self._is_on = msg.data

    def __callback(self, msg):
        """
        Callback to the ROS subscriber. 

        Decodes the arduino msg

        Parameters
        ----------
        msg : std_msgs/UInt8
            The readings of the sensor
        
        """
        # Check if at least one sensor is over the line 
        #sensorValueA = np.array( [ (msg.data & 2) != 0, (msg.data & 8) != 0, (msg.data & 16) != 0, (msg.data & 64) != 0])
        #sensorValueB = np.array([ (msg.data & 1) != 0, (msg.data & 4    ) != 0, (msg.data & 32) != 0, (msg.data & 128) != 0])
        sensorValueA = self._readings[::2]   >  20
        sensorValueB = self._readings[1::2]  >  20

        #self._readings = (self._readings < 5) * self._readings

       
        readings =  2 * (self._readings[1::2]) / (self._readings[::2] + self._readings[1::2]) - 1

        onLine =  sensorValueA + sensorValueB

        for i in range(4):
            if (onLine[i]):
                self._error[i] = readings[i]
            else:
                self._error[i] = 1 if self._error[i] > 0 else -1

    def reset(self, resetToMinimun=True):
        """
        Reset the tracking of the line. This is useful to start alinging with a new line

        Parameters
        ----------
        resetToMinimun : boolean (default True)
            if True, the new values are -2, otherwise they are the 2
        """

        for i in range(len(self._error)):
            if resetToMinimun:
                self._error = np.ones(4)*-2
            else:
                self._error = np.ones(4)*2

    def readLines(self):
      
        return self._error
    
    def __del__(self):

        self.pub_enable.publish(False)  
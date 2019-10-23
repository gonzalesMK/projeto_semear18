#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt8, Bool
import numpy as np
from enum import Enum
from enum import IntEnum
from functools import partial

import serial
import struct

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

        while( not rospy.is_shutdown() ):
            try:
                self.arduino = serial.Serial('/dev/ttyUSB0', 115200, timeout=.01)
                break
            except:
                rospy.loginfo("Trying again ttyUSB0")
                rospy.Rate(1).sleep()

        while( not rospy.is_shutdown()):
            a = struct.pack('!B', 68)
            self.arduino.write(a)
            a = str(self.arduino.read(14))

            b = np.zeros(14)            
            if len(a) == 14:
                break
        self.arduino.reset_input_buffer()
        self.arduino.reset_output_buffer()
        self._is_on = False
        print("Turning on sensors")

        
    def _read(self):
        a = struct.pack('!B', 68)
        self.arduino.write(a)
        a = str(self.arduino.read(14))

        b = np.zeros(14)            
        if len(a) == 14:
            j = 0
            for i in a:
                b[j] = ord( i)
                j = j + 1
            
        readings = b[0:8]
        # rospy.loginfo("readings: {}".format(readings))
        self._max_readings = np.maximum(self._max_readings, readings)
        self._min_readings = np.minimum(self._min_readings, readings)
        self._readings =  (readings - self._min_readings)/(self._max_readings - self._min_readings) * 100
        self.container = b[8]
        self.limit_switchs = b[9]
        self.encoder =  -( b[10] + b[11] * 256 + b[12] * 65536 + b[13] * 16777216 - 2147483648) 

        sensorValueA = self._readings[::2]   >  20
        sensorValueB = self._readings[1::2]  >  20
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
                self._error = np.ones(4)*-1
            else:
                self._error = np.ones(4)*1

    def readLines(self):
        self._read()
        return self._error
    
    def __del__(self):

        self.arduino.close()
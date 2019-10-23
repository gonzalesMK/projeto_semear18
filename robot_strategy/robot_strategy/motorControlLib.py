#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import serial
import struct
from std_msgs.msg import Float64, Bool
import numpy as np
from enum import Enum
from lineSensors import Sides
from enum import IntEnum
import copy

"""
    FL = 0
    FR = 1
    BL = 2
    BR = 3

"""
class Wheels(IntEnum):
    FL = 0
    FR = 1
    BL = 2
    BR = 3

    def __int__(self):
        return self.value

class MotorControl(object):
    MEDIUM_VEL = 1
    def __init__(self, Kp=0.0, Kd=0.0, Ki = 0, windUp = 0, deadSpace = 15, freq=100.0, momentum=0):
        """
            momentum é a porcentagem do derivativo que deve ser mantida depois de 1 segundo 

        """
        self.Kp = float(Kp)
        self.Kd = float(Kd)
        self.pid_on = False
        self.freq = float(freq)
        self.momentum =  float(momentum)
        self.Ki = float(Ki)
        self.windUp = float(windUp)
        self.deadSpace = float(deadSpace)

        self.arduino = serial.Serial('/dev/ttyUSB1', 115200, timeout=.01)


    def setVelocity(self, actuation):
        self.arduino.write( struct.pack(">4B", actuation[Wheels.FL], actuation[Wheels.FR],actuation[Wheels.BL],actuation[Wheels.BR] ))

    def align(self, error_array, target=0 ):

        if not self.pid_on:
            self.pid_on = True
            self.deltaError = np.array([0, 0, 0, 0])
            self.integrative = np.array([0., 0., 0., 0.])
            self.pastError = copy.deepcopy(error_array)
            return

        #self.pub_motorLineFL.publish( error_array[int(Wheels.FL)])
        #self.pub_motorLineFR.publish( error_array[int(Wheels.FR)])
        #self.pub_motorLineBL.publish( error_array[int(Wheels.BL)])
        #self.pub_motorLineBR.publish( error_array[int(Wheels.BR)])
       
        self.deltaError = (error_array - self.pastError)  + self.momentum * self.deltaError
        
        self.integrative += error_array / self.freq

        for n in range(4):
            
            if np.abs(self.integrative[n]) * np.abs(self.Ki) > self.windUp :

                self.integrative[n] = self.windUp/np.abs(self.Ki) * np.sign(self.integrative[n])

        actuation = error_array * self.Kp + self.deltaError * self.Kd + self.Ki * self.integrative
        # rospy.loginfo("\nError Array: {} \nPast Error: {}\nDeltaError: {} \nActuation: {} ".format(error_array, self.pastError, self.deltaError, actuation ) + 
        # "\nP:{}\nD: {}\nI: {}".format(self.Kp*error_array, self.deltaError * self.Kd, self.Ki*self.integrative)
        # )
        
        #rospy.loginfo("Actuation {}".format(actuation))
        
        if max(abs(actuation)) > 120:
            actuation = actuation / ( max(abs(actuation)) / 120. )
            
        if  sum(error_array == 0) != 4 and  np.abs(max(actuation)) < self.deadSpace and np.abs(max(actuation)) > 0.1:
            actuation = self.deadSpace * ( np.abs(actuation) > 0.1 ) * np.sign(actuation)

        # self.pub_motorPWM_FL.publish( actuation[int(Wheels.FL)])
        # self.pub_motorPWM_FR.publish( actuation[int(Wheels.FR)])
        # self.pub_motorPWM_BL.publish( actuation[int(Wheels.BL)])
        # self.pub_motorPWM_BR.publish( actuation[int(Wheels.BR)])

        actuation = actuation + 120
        #rospy.loginfo("{}".format(actuation))
        self.arduino.write( struct.pack(">4B", actuation[Wheels.FL],actuation[Wheels.FR], actuation[Wheels.BL], actuation[Wheels.BR]))
        #b = 0
        #b = self.arduino.read(1)
        #if len(b):
        #    rospy.loginfo(ord(b))
        self.pastError = copy.deepcopy(error_array)

    def clear(self):
        self.pid_on = False

    def setParams(self, Kp=None, Kd=None, freq=None, momentum=None, Ki=None, windUp=None, deadSpace=None):
        
        if not (Kp is None) :
            self.Kp = float(Kp)

        if not (Kd is None) :
            self.Kd = float(Kd)

        if not (freq is None) :
            self.freq = float(freq)

        if not (momentum is None) :
            self.momentum = float(momentum)

        if not (Ki is None):
            self.Ki = float(Ki)
        
        if not (windUp is None):
            self.windUp = float(windUp)

        if not (deadSpace is None):
            self.deadSpace = deadSpace

    def stop(self):

        self.arduino.write(struct.pack(">4B", 120, 120, 120, 120))

        rospy.Rate(10).sleep()  

    def __del__(self):
        self.stop()
        

   
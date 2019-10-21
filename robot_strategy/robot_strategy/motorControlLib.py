#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
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

        # This PID is to control velocity
        self.pub_motorFL = rospy.Publisher('/motorFL/desired_vel', Float64, queue_size=10)    
        self.pub_motorFR = rospy.Publisher('/motorFR/desired_vel', Float64, queue_size=10)    
        self.pub_motorBL = rospy.Publisher('/motorBL/desired_vel', Float64, queue_size=10)    
        self.pub_motorBR = rospy.Publisher('/motorBR/desired_vel', Float64, queue_size=10)

        self.pub_encoderEnable = rospy.Publisher('/encoder_enable', Bool, queue_size=10)  

        # This PID is to align with a line 
        self.pub_motorLineFL = rospy.Publisher('/motorSensorFL/error', Float64, queue_size=10)    
        self.pub_motorLineFR = rospy.Publisher('/motorSensorFR/error', Float64, queue_size=10)    
        self.pub_motorLineBL = rospy.Publisher('/motorSensorBL/error', Float64, queue_size=10)    
        self.pub_motorLineBR = rospy.Publisher('/motorSensorBR/error', Float64, queue_size=10)

        self.pub_motorPWM_FL = rospy.Publisher('/motorFL/pwm', Float64, queue_size=10)    
        self.pub_motorPWM_FR = rospy.Publisher('/motorFR/pwm', Float64, queue_size=10)    
        self.pub_motorPWM_BL = rospy.Publisher('/motorBL/pwm', Float64, queue_size=10)    
        self.pub_motorPWM_BR = rospy.Publisher('/motorBR/pwm', Float64, queue_size=10)
        
        self.pub_lineEnable = rospy.Publisher( '/pid_enable', Bool, queue_size=10)
        self.sub_lineEnable = rospy.Subscriber('/pid_enable', Bool, self.__pid_enable_cb)    
        self.pub_lineTarget = rospy.Publisher('/desired_pose', Float64, queue_size=10)   

        rospy.Rate(2).sleep()  

        self._velocity_mode = False
        self._align_mode = False

    def __pid_enable_cb(self, msg):
        """ We had a problem while publishing the pid_enable topic. Sometimes it would just not arrive... 
       
        So, now, we're going to publish the enable topic until we got a response
        """
        self._velocity_mode = True


    def setVelocityControlMode(self):
        
        self._velocity_mode = False
        
        while (not rospy.is_shutdown()) and (not self._velocity_mode): 
            self.pub_encoderEnable.publish(True)
            self.pub_lineEnable.publish(False)
            rospy.Rate(0.5).sleep()  

        #self._velocity_mode = True
        self._align_mode = False

    def setAlignControlMode(self, target=0, side=Sides.LEFT):
        
        self.pub_encoderEnable.publish(False)
        self.pub_lineEnable.publish(True)

        self._velocity_mode = False
        self._align_mode = True

        self.pub_lineTarget.publish(target)

    
    def setVelocity(self, vel):
        
        if not self._velocity_mode:
            self.setVelocityControlMode()
        
        self.pub_motorFL.publish(vel[int(Wheels.FL)])
        self.pub_motorFR.publish(vel[int(Wheels.FR)])
        self.pub_motorBL.publish(vel[int(Wheels.BL)])
        self.pub_motorBR.publish(vel[int(Wheels.BR)])

    def align(self, error_array, target=0 ):

        if not self.pid_on:
            self.pid_on = True
            self.deltaError = np.array([0, 0, 0, 0])
            self.integrative = np.array([0., 0., 0., 0.])
            self.pastError = copy.deepcopy(error_array)
            return

        if not self._align_mode:
            self.setAlignControlMode(target)
        
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

        self.pub_motorPWM_FL.publish( actuation[int(Wheels.FL)])
        self.pub_motorPWM_FR.publish( actuation[int(Wheels.FR)])
        self.pub_motorPWM_BL.publish( actuation[int(Wheels.BL)])
        self.pub_motorPWM_BR.publish( actuation[int(Wheels.BR)])

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

    def setPIDVelocity(self, vel_array, base_vel, side= Sides.FRONT):
        if not self._velocity_mode:
            self.setVelocityControlMode()

        if( Sides.FRONT == side):
            vel_array[int(Wheels.FR)] += base_vel
            vel_array[int(Wheels.BR)] += base_vel
            vel_array[int(Wheels.FL)] += base_vel
            vel_array[int(Wheels.BL)] += base_vel
        elif( Sides.BACK == side):
            vel_array[int(Wheels.FR)] -= base_vel
            vel_array[int(Wheels.BR)] -= base_vel
            vel_array[int(Wheels.FL)] -= base_vel
            vel_array[int(Wheels.BL)] -= base_vel                        
        elif( Sides.LEFT == side):
            vel_array[int(Wheels.FR)] += base_vel
            vel_array[int(Wheels.BR)] -= base_vel
            vel_array[int(Wheels.FL)] -= base_vel
            vel_array[int(Wheels.BL)] += base_vel
        elif( Sides.LEFT == side):
            vel_array[int(Wheels.FR)] -= base_vel
            vel_array[int(Wheels.BR)] += base_vel
            vel_array[int(Wheels.FL)] += base_vel
            vel_array[int(Wheels.BL)] -= base_vel

        self.pub_motorFL.publish(vel_array[int(Wheels.FL)])
        self.pub_motorFR.publish(vel_array[int(Wheels.FR)])
        self.pub_motorBL.publish(vel_array[int(Wheels.BL)])
        self.pub_motorBR.publish(vel_array[int(Wheels.BR)])

    def stop(self):
        self.pub_motorPWM_FL.publish(0)
        self.pub_motorPWM_FR.publish(0)
        self.pub_motorPWM_BL.publish(0)
        self.pub_motorPWM_BR.publish(0)
        
        self.pub_encoderEnable.publish(False)
        self.pub_lineEnable.publish(False)

        self.pub_motorFL.publish(0)
        self.pub_motorFR.publish(0)
        self.pub_motorBL.publish(0)
        self.pub_motorBR.publish(0)

        self.pub_motorLineFL.publish(0)
        self.pub_motorLineFR.publish(0)
        self.pub_motorLineBL.publish(0)
        self.pub_motorLineBR.publish(0)

        rospy.Rate(10).sleep()  

        # Publish 0 To the PWM topics in order to stop the robot definitively
        self.pub_motorPWM_FL.publish(0)
        self.pub_motorPWM_FR.publish(0)
        self.pub_motorPWM_BL.publish(0)
        self.pub_motorPWM_BR.publish(0)
        
        self._velocity_mode = False
        self._align_mode = False    

#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, Bool
import numpy as np
from enum import Enum
from lineSensors import Sides
from enum import IntEnum
class Wheels(IntEnum):
    FL = 0
    FR = 1
    BL = 2
    BR = 3

    def __int__(self):
        return self.value

class MotorControl(object):
    MEDIUM_VEL = 1
    def __init__(self):

        # This PID is to control velocity
        self.pub_motorFL = rospy.Publisher('/motorFL/desired_vel', Float64, queue_size=10)    
        self.pub_motorBL = rospy.Publisher('/motorBL/desired_vel', Float64, queue_size=10)    
        self.pub_motorFR = rospy.Publisher('/motorFR/desired_vel', Float64, queue_size=10)    
        self.pub_motorBR = rospy.Publisher('/motorBR/desired_vel', Float64, queue_size=10)

        self.pub_encoderEnable = rospy.Publisher('/encoder_enable', Bool, queue_size=10)  

        # This PID is to align with a line 
        self.pub_motorLineFL = rospy.Publisher('/motorSensorFL/error', Float64, queue_size=10)    
        self.pub_motorLineFR = rospy.Publisher('/motorSensorFR/error', Float64, queue_size=10)    
        self.pub_motorLineBL = rospy.Publisher('/motorSensorBL/error', Float64, queue_size=10)    
        self.pub_motorLineBR = rospy.Publisher('/motorSensorBR/error', Float64, queue_size=10)

        self.pub_motorPWM_FL = rospy.Publisher('/motorFL/pwm', Float64, queue_size=10)    
        self.pub_motorPWM_FR = rospy.Publisher('/motorBL/pwm', Float64, queue_size=10)    
        self.pub_motorPWM_BL = rospy.Publisher('/motorFR/pwm', Float64, queue_size=10)    
        self.pub_motorPWM_BR = rospy.Publisher('/motorBR/pwm', Float64, queue_size=10)
        
        self.pub_lineEnable = rospy.Publisher('/pid_enable', Bool, queue_size=10)    
        self.pub_lineTarget = rospy.Publisher('/desired_pose', Float64, queue_size=10)   

        rospy.Rate(2).sleep()  

        self._velocity_mode = False
        self._align_mode = False

    def setVelocityControlMode(self):
        
        self.pub_encoderEnable.publish(True)
        self.pub_lineEnable.publish(False)

        self._velocity_mode = True
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
        if not self._align_mode:
            self.setAlignControlMode(target)
        
        self.pub_motorLineFL.publish(error_array[int(Wheels.FL)])
        self.pub_motorLineFR.publish(error_array[int(Wheels.FR)])
        self.pub_motorLineBL.publish(error_array[int(Wheels.BL)])
        self.pub_motorLineBR.publish(error_array[int(Wheels.BR)])

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
        self.setVelocity([0,0,0,0])

        self.pub_motorFL.publish(0)
        self.pub_motorFR.publish(0)
        self.pub_motorBL.publish(0)
        self.pub_motorBR.publish(0)

        self.pub_motorLineFL.publish(0)
        self.pub_motorLineFR.publish(0)
        self.pub_motorLineBL.publish(0)
        self.pub_motorLineBR.publish(0)

        rospy.Rate(10).sleep()  

        self.pub_encoderEnable.publish(False)
        self.pub_lineEnable.publish(False)

        # Publish 0 To the PWM topics in order to stop the robot definitively
        self.pub_motorPWM_FL.publish(0)
        self.pub_motorPWM_FR.publish(0)
        self.pub_motorPWM_BL.publish(0)
        self.pub_motorPWM_BR.publish(0)
        
        self._velocity_mode = False
        self._align_mode = False    

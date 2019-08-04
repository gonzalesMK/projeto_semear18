#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt8, Float64, Bool
import numpy as np
from enum import Enum
from std_msgs.msg import Float64, Bool
import numpy as np


class GearAndPinionPoses(Enum):
    container1 = 1000
    container2 = 2000
    container3 = 3000
    container4 = 4000
    container5 = 5000
    container6 = 6000

    def __int__(self):
        return self.value

class ServoPose(Enum):
    Left = 10
    Right = 170
    Center = 100

    def __int__(self):
        return self.value

class Claw(object):
    """
    """

    def __init__(self):

        self._subscribers = rospy.Subscriber("/containerSensors", UInt8, self.__callback)
        self.sensor = [0, 0]

        rospy.Subscriber("/clawEncoder", Float64, self.__clawEncoderCB)
        
        self.pubServoPose = rospy.Publisher("/setServoPose", Float64, queue_size=1) # Control the Position of the Servo Motor
        
        #self.pubClawDesiredVel = rospy.Publisher("/setClawVel", Float64, 1)# Control the velocity of the Gear and Pinion
        self.pubClawDesiredVel = rospy.Publisher("/claw/desired_vel", Float64, 1)
        self.pubClawControllerEnable = rospy.Publisher('/claw/pid_enable', Bool, queue_size=10)    
        self.pubClawControllerError = rospy.Publisher('/claw/error', Bool, queue_size=10)    
        self.pubClawPWM = rospy.Publisher("/setClawPWM", Float64, 1)
        self.pubTurnOnClawFeedback = rospy.Publisher("/turnOnClaw", Bool, 1)# Turn on the Gear and Pinion arduino interface
        self.pubTurnOnElectro = rospy.Publisher("/turnOnElectromagnet", Bool, 1)# Turn on the Electromagnet

        self.__precision = 100
        self.__Kp = 0.0001
        self.__limitVel = 0.1

    def __clawEncoderCB(self, msg):
        """
        Callback to the ROS subscriber

        Parameters
        ----------
        msg : std_msgs/UInt8
            The readings of the Claw's Encoder

        """

        self.encoder = msg.data

    def __clipVel(self, vel):
        
        if np.abs(vel) > self.__limitVel:
            vel = self.__limitVel * vel / np.abs(vel)

        return vel    

    def __turnOnGearAndPinion(self):
        """
            Turn on the Encoder Feedback from the arduino
            Turn on the Controller for the Gear and Pinnion
        """
        self.pubTurnOnClawFeedback.publish(True)
        self.pubClawControllerEnable(True) 
        
    def __turnOffGearAndPinion(self):
        """
            Turn off the Encoder Feedback from the arduino
            Turn off the Controller for the Gear and Pinnion
            Set PWM to 0 to be sure that the claw is not moving
        """
        self.pubTurnOnClawFeedback.publish(False)
        self.pubClawControllerEnable(False) 
        self.pubClawPWM(0)
        
    def setGearAndPinionPose(self, target):
        """
            Controls the Gear and Pinion position with care:
                - When it goes down/iup, it needs to start fast and slow down before arriving to the target - otherwise, because of 
                the inertia, the Gear and Pinion are going to hit something

            Target: should be one of GearAndPinionPoses
        """
        self.__turnOnGearAndPinion()

        error = self.encoder - target

        while( np.abs(error) > self.__precision):
            
            error = self.encoder - target

            vel = self.__clipVel(error * self.__Kp)
            
            self.pubClawDesiredVel(vel)

        self.__turnOffGearAndPinion()

    def setServoPose(self, pose):
         
         self.pubServoPose.publishe( int(pose) )


        



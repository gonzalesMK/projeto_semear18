#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import UInt8, Float64, Bool, Int64
import numpy as np
from enum import Enum
from std_msgs.msg import Float64, Bool
import numpy as np


class GearAndPinionPoses(Enum):
    container1 = 4.0 # This is the lowest container
    container2 = 8.0
    container3 = 12.0 
    container4 = 16.0 # This is the highest container in the Dock
    container5 = 20.0
    container6 = 24.0
    reset = 26.32

    def __float__(self):
        return self.value

class ServoPose(Enum):
    Left = 0
    Center = 90
    Right = 180
    
    def __int__(self):
        return self.value

class Claw(object):
    """
    This class configures all the Topics for the Claw and provides a practical interface:

   Contagens por revolução: 8245.92 counts per revolution quando se usa os 2 canais subida e descida. No nosso caso, temos 4122.96 voltas. O diâmetro primitivo e 18mm

    Publish:
        /claw/enableFB: Bool
            turn On the DC Motor's encoder feedback from the Arduino.
        
        /claw/enableElectroMagnet: Bool
             turn On the Electromagnet
        
        /claw/pwm: Float64
            the PWM that is sent to the DC motor in the Arduino. We set this to 0 to stop completly the Gear and Pinion.
                    Also, the controller use this as output

        ---------- if using external controller
        /claw/desired_vel : Float64
            the desired vel of the Gear and Pinion. The controller uses this topic as input
        
        /claw/pid_enable  : Bool
            turn on the Controller
        
       
    Subscribe to:
        /claw/limitSwitch: Bool
            the upperlimit of the Gear and Pinion. True is activated (the Gear is in the lmit position)
        
        /claw/height: Float64 (centimeters)
            the position of the Gear and Pinion in cm

        -------------- if using external controller ---------
        /claw/error:  Bool
            the error in the position of the Gear and Pinion. The controller uses this topic as input
        

    Important Constants:
        precision: the precision of the Gear and Pinion position
        limitVel: the limits of the Gear and Pinion velocity
        fastVel: the velocity in the faster (starting) phase
        slowVel: the velocity in the slower (arriving) phase
        controllerFreq: the frequency to Control the DC Motor
    """

    def __init__(self):
        
        #self._subscribers = rospy.Subscriber("/containerSensors", UInt8, self.__callback)
        #self.sensor = [0, 0]

        rospy.Subscriber("/claw/height", Int64, self.__clawEncoderCB)
        rospy.Subscriber("/claw/limitSwitch", Bool, self.__clawSwitchCB)
        
        self.pubServoPose = rospy.Publisher("/claw/servoPose", UInt8, queue_size=1) # Control the Position of the Servo Motor
        self.pubClawPWM = rospy.Publisher("/claw/pwm", Float64, queue_size=1)
        self.pubTurnOnClawFeedback = rospy.Publisher("/claw/turnOnFB", Bool, queue_size=1)# Turn on the Gear and Pinion arduino interface
        self.pubTurnOnElectro = rospy.Publisher("/claw/turnOnElectromagnet", Bool, queue_size=1)# Turn on the Electromagnet
        
        
        self.pubClawDesiredVel = rospy.Publisher("/claw/desired_vel", Int64, queue_size=1)
        #self.pubClawControllerEnable = rospy.Publisher('/claw/pid_enable', Bool, queue_size=10)    
        #self.pubClawControllerError = rospy.Publisher('/claw/error', Bool, queue_size=10)    
        

        self.__precision = 0.01 
        self.__limitVel = 0.1
        self.__fastVel = 254.0
        self.__slowVel = 70.0
        self.__controllerFreq = 50.0
        self.__timer = rospy.Rate(self.__controllerFreq)
        self.startPose = 0.0
        self.is_referenced = True
    
    def __clawEncoderCB(self, msg):
        """
        Callback to the ROS subscriber

        Parameters
        ----------
        msg : std_msgs/UInt8
            The readings of the Claw's Encoder converted to height in cm. The reference position is the ground.

        """

        self.gearAndPinionHeight = msg.data / 4122.96 * ( 2.0 * np.pi * 1.8) / 2.0 - self.startPose #  This converts pulses to cm.

    def __clawSwitchCB(self, msg):

        self.upperLimit = msg.data & 4
        self.lowerLimit = msg.data & 1

    def __clipVel(self, vel):
        """
            Set limits for the Velocity
        """
        if np.abs(vel) > self.__limitVel:
            vel = self.__limitVel * vel / np.abs(vel)

        return vel    

    def __clipRackPose(self, pose):
        """
            Set limits for the Rack pose
        """
        if pose < 0 : 
            pose = 0
        
        if pose > self.__limitRackPose:
            pose = self.__limitRackPose

        return pose    

    def setGearAndPinionPose(self, target, pickContainer=False):
        """
            Controls the Gear and Pinion position with care using my own control:
                - When it goes down/iup, it needs to start fast and slow down before arriving to the target - otherwise, because of 
                the inertia, the Gear and Pinion are going to hit something

            Target: should be one of {1, 2, 3, 4, 5, 6 }:
                This is the height of the container: is it the first (1), second(2), .. ? 

            pickContainer: Boolean, (default=False):
                If this is set to True, the claw is going to go down'til the button in the claw is pressed by touching the desired container.
        """
        try: 
            target = float( GearAndPinionPoses(target) )
        except ValueError:
            return 

        if not self.is_referenced:
            rospy.logerr("A cremalheira não foi referenciada. Cuidado! Chamar a função <> antes de usar o set Pose")

        error = self.gearAndPinionHeight - target

        timer = rospy.Rate(50)
        while( np.abs(error) > self.__precision):
            
            error = self.gearAndPinionHeight - target # in cm
            rospy.loginfo(error)
            
            if np.abs(error) > 1:
                self.pubClawPWM.publish( self.__fastVel * np.sign(error) )
            else:
                self.pubClawPWM.publish( self.__slowVel * np.sign(error) )
            
            timer.sleep()


        if( pickContainer):
            touched = False
            while( not touched ):
                #TODO: 
                self.pubClawDesiredVel(self.__slowVel)
                self.timer.sleep()
                #subscribe to the right topic and wait for it to publish true:
                pass

        self.pubClawPWM.publish(0)

    def setGearAndPinionPosePID(self, target):
        """
            Controls the Gear and Pinion position with care using a Controller node:
                - When it goes down/iup, it needs to start fast and slow down before arriving to the target - otherwise, because of 
                the inertia, the Gear and Pinion are going to hit something

            Target: should be one of GearAndPinionPoses
        """
        error = self.gearAndPinionHeight - target

        if not self.is_referenced:
            rospy.logerr("A cremalheira não foi referenciada. Cuidado! Chamar a função <> antes de usar o set Pose")

        while( np.abs(error) > self.__precision):
            
            error = self.gearAndPinionHeight - target

            vel = self.__clipVel(error * self.__Kp)
            
            self.pubClawDesiredVel(vel)

            self.timer.sleep()

    
    def setServoPose(self, pose):
         """
            Set the Servomotor pose.

            Pose should be the enum ServoPose to standardize the height
         """
         self.pubServoPose.publish( int(pose) )


    def resetGearAndPinionPose(self):
        """
            Resets the height of the Gear and Pinion. This is useful to reference the Gear and Pinion to the original position and 
            thus set the integrative error of the encoder to 0

            This pose is also useful to carry the container around
         """
        while( self.upperLimit ):
            self.pubClawPWM( 62 )
        
        self.pubClawPWM( 0 )

        """
            Resets the arduino reference for the Height again to the start position
        """
        self.startPose = self.gearAndPinionHeight

        self.is_referenced = True
        

    def controlElectromagnet(self, turnOn = True):
        self.pubTurnOnElectro.publish(turnOn)

        


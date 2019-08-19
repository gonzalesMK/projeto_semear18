#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt8, Float64, Bool
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

    def __int__(self):
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
        
        self._subscribers = rospy.Subscriber("/containerSensors", UInt8, self.__callback)
        self.sensor = [0, 0]

        rospy.Subscriber("/claw/height", Float64, self.__clawEncoderCB)
        rospy.Subscriber("/claw/limitSwitch", Bool, self.__clawSwitchCB)
        
        self.pubServoPose = rospy.Publisher("/claw/servoPose", Float64, queue_size=1) # Control the Position of the Servo Motor
        self.pubClawPWM = rospy.Publisher("/claw/PWM", Float64, 1)
        self.pubTurnOnClawFeedback = rospy.Publisher("/claw/turnOnFB", Bool, 1)# Turn on the Gear and Pinion arduino interface
        self.pubTurnOnElectro = rospy.Publisher("/claw/turnOnElectromagnet", Bool, 1)# Turn on the Electromagnet
        
        
        #self.pubClawDesiredVel = rospy.Publisher("/claw/desired_vel", Float64, 1)
        #self.pubClawControllerEnable = rospy.Publisher('/claw/pid_enable', Bool, queue_size=10)    
        #self.pubClawControllerError = rospy.Publisher('/claw/error', Bool, queue_size=10)    
        

        self.__precision = 0.01 
        self.__limitVel = 0.1
        self.__fastVel = 255 
        self.__slowVel = 100
        self.__controllerFreq = 50 
        self.__timer = rospy.Rate(self.__controllerFreq)
    
    def __clawEncoderCB(self, msg):
        """
        Callback to the ROS subscriber

        Parameters
        ----------
        msg : std_msgs/UInt8
            The readings of the Claw's Encoder converted to height in cm. The reference position is the ground.

        """

        self.gearAndPinionHeight = msg.data

    def __clawSwitchCB(self, msg):

        self.limitSwitch = msg.data

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

    def __turnOnGearAndPinion(self):
        """
            Turn on the Encoder Feedback from the arduino
            Turn on the Controller for the Gear and Pinnion
        """
        self.gearAndPinionHeight = -1
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
            Controls the Gear and Pinion position with care using my own control:
                - When it goes down/iup, it needs to start fast and slow down before arriving to the target - otherwise, because of 
                the inertia, the Gear and Pinion are going to hit something

            Target: should be one of GearAndPinionPoses
        """
        self.__turnOnGearAndPinion()
        target = self.__clipRackPose(target)
        error = self.gearAndPinionHeight - target

        while( np.abs(error) > self.__precision):
            
            error = self.gearAndPinionHeight - target # in cm

            if np.abs(error) > 5:
                self.pubClawDesiredVel(self.__fastVel)
            else:
                self.pubClawDesiredVel(self.__slowVel)
            
            self.timer.sleep()

    def setGearAndPinionPosePID(self, target):
        """
            Controls the Gear and Pinion position with care using a Controller node:
                - When it goes down/iup, it needs to start fast and slow down before arriving to the target - otherwise, because of 
                the inertia, the Gear and Pinion are going to hit something

            Target: should be one of GearAndPinionPoses
        """
        self.__turnOnGearAndPinion()

        error = self.gearAndPinionHeight - target


        while( np.abs(error) > self.__precision):
            
            error = self.gearAndPinionHeight - target

            vel = self.__clipVel(error * self.__Kp)
            
            self.pubClawDesiredVel(vel)

            self.timer.sleep()

        self.__turnOffGearAndPinion()
    
    def setServoPose(self, pose):
         """
            Set the Servomotor pose.

            Pose should be the enum ServoPose to standardize the height
         """
         self.pubServoPose.publishe( int(pose) )

    def __resetHeight(self):
        """
            Resets the arduino reference for the Height again to the start position
        """
        #TODO: here
        pass
 
    def resetGearAndPinionPose(self):
        """
            Resets the height of the Gear and Pinion. This is useful to reference the Gear and Pinion to the original position and 
            thus set the integrative error of the encoder to 0

            This pose is also useful to carry the container around

         """
        self.__turnOnGearAndPinion()

        error = self.gearAndPinionHeight - (GearAndPinionPoses.reset + self.__referencePoseError)

        while( limitSwitch == False ):
            
            error = self.gearAndPinionHeight - GearAndPinionPoses.reset

            if np.abs(error) > 5:
                self.pubClawPWM(self.__fastPWM)
            else:
                self.pubClawPWM(self.__slowPWM)
            
            self.timer.sleep()

        self.pubClawPWM(0)
        for i in range(10):
            self.timer.sleep()
        
        # Set new reference for position
        self.__resetHeight()


        


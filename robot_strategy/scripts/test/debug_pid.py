#! /usr/bin/env python
import rospy
import actionlib
from std_msgs.msg import Float64, Bool
from robot_strategy.motorControlLib import MotorControl, Wheels
from robot_strategy.statesLib import finalAlignmentIntersection
from robot_strategy.containerSensorsLib import ContainerSensors
from geometry_msgs.msg import Twist

from robot_strategy.lineSensors import Sides, LineSensor

import numpy as np

"""
Interface to control the robot base with the teleop_twist_keyboard

"""
x=0
y=0
yaw=0

motorControl=0


if __name__ == '__main__':
    rospy.init_node('testeMotor')
    
    motorControl = MotorControl(kd= 0.1, Kp = 15, momentum=1)
    rospy.loginfo(motorControl.momentum)
    r = rospy.Rate(100)

    change = 0
    error = np.array( [1,1,1,1 ])

    while( not rospy.is_shutdown() ):
        change += 1

        if change == 300:
            error = error * -1
            #change = 301 
        
        motorControl.align(error)

        r.sleep()

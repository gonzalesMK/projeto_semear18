#! /usr/bin/env python
import rospy
import actionlib
from std_msgs.msg import Float64, Bool
from robot_strategy.motorControlLib import MotorControl, Wheels

from geometry_msgs.msg import Twist

from robot_strategy.lineSensors import Sides

import numpy as np

from functools import partial
"""
Interface to control the robot base with the teleop_twist_keyboard

"""
x=0
y=0
yaw=0


def cmdvel_cb(motorControl, msg):
    vel = np.array([0,0,0,0], dtype=float)

    LDIAG = 0.116383204
    DIAMETRO = 0.099060

    x = msg.linear.x
    y = msg.linear.y
    yaw = msg.angular.z

    theta = np.arctan2(y, x)
    w_module = np.sqrt(x**2 + y**2) / DIAMETRO
    conversao = LDIAG/DIAMETRO

    vel[int(Wheels.FL)] = (w_module)*np.cos(np.pi / 4 - theta) - (yaw) * 20
    vel[int(Wheels.FR)] = (w_module)*np.cos(np.pi / 4 + theta) + (yaw) * 20
    vel[int(Wheels.BL)] = (w_module)*np.cos(np.pi / 4 + theta) - (yaw) * 20
    vel[int(Wheels.BR)] = (w_module)*np.cos(np.pi / 4 - theta) + (yaw) * 20

    motorControl.align(vel)        


if __name__ == '__main__':
    rospy.init_node('testeMotor')
    
    motorControl = MotorControl()
    motorControl.setParams(Kp=20., Kd=0., freq=100, momentum=0.6)
    rospy.Rate(1).sleep()
    vel = np.array([0,0,0,0])
    rospy.Subscriber( "/cmd_vel", Twist, partial(cmdvel_cb, motorControl))
    
    rospy.Rate(1).sleep()        
    motorControl.align(vel)
    rospy.loginfo("done")
    rospy.spin()
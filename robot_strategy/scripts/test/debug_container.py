#! /usr/bin/env python
import rospy
import actionlib
from std_msgs.msg import Float64, Bool
from robot_strategy.motorControlLib import MotorControl, Wheels
from robot_strategy.statesLib import finalAlignmentIntersection
from robot_strategy.containerSensorsLib import ContainerSensors
from geometry_msgs.msg import Twist

from robot_strategy.lineSensors import Sides, LineSensor
#from robot_strategy.statesLib import , changeIntersection, goToDock, goFromDockToIntersection
from robot_strategy.statesLib import change_intersection, goToFirstPose, dock_to_intersection, to_dock, to_container, container_to_intersection, firstPose
from robot_strategy.utils import Colors, Positions

import numpy as np

import sys, signal
from functools import partial

def signal_handler(motorControl, signal, frame):
    print("\nprogram exiting gracefully")
    motorControl.stop()
    print("\nprogram exiting gracefully")
    sys.exit(0)



"""
Interface to control the robot base with the teleop_twist_keyboard

"""
x=0
y=0
yaw=0

motorControl=0
class UserData(object):
    finalPose = 0
    robotPose = 0
    containerColor = 0
    def __init__(self):
        pass

def cmdvel_cb(msg):
    global motorControl
    vel = [0,0,0,0]

    LDIAG = 0.116383204
    DIAMETRO = 0.099060

    x = msg.linear.x
    y = msg.linear.y
    yaw = msg.angular.z

    theta = np.arctan2(y, x)
    w_module = np.sqrt(x**2 + y**2) / DIAMETRO
    conversao = LDIAG/DIAMETRO

    vel[int(Wheels.FL)] = ((w_module)*np.sin(np.pi / 4 + theta) + (yaw) * conversao) * np.pi/10; # Rad/s
    vel[int(Wheels.FR)] = ((w_module)*np.cos(np.pi / 4 + theta) - (yaw) * conversao) * np.pi/10; # Rad/s
    vel[int(Wheels.BL)] = ((w_module)*np.cos(np.pi / 4 + theta) + (yaw) * conversao) * np.pi/10; # Rad/s
    vel[int(Wheels.BR)] = ((w_module)*np.sin(np.pi / 4 + theta) - (yaw) * conversao) * np.pi/10; # Rad/s

    if isinstance(motorControl, MotorControl):
        motorControl.setVelocity(vel)        


if __name__ == '__main__':
    rospy.init_node('testeMotorLib')
    
    motorControl = MotorControl(Kp=0.0, Kd= 0.0, momentum= 0.)
    
    signal.signal(signal.SIGINT, partial(signal_handler, motorControl), )
    containerSensors = ContainerSensors()
    rospy.Subscriber( "/cmd_vel", Twist, cmdvel_cb)
    
    linesensors = LineSensor()
    error = np.array([0,0,0,0])
    sensors = linesensors.readLines()

    to_container(linesensors, motorControl, containerSensors)
    
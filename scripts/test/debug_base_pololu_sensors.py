#! /usr/bin/env python
import rospy
import actionlib
from std_msgs.msg import Float64, UInt8

from projeto_semear.lineSensors import LineSensor, Sides
from projeto_semear.containerSensorsLib import ContainerSensors
from projeto_semear.motorControlLib import MotorControl, Wheels
from projeto_semear.utils import Colors


from projeto_semear.lineSensors import Sides

import numpy as np

"""
Because all the sensors are transmited in just one topic bit by bit, it is handy to have a code to debug it:


"""
def pololu_cb(msg):


if __name__ == '__main__':
    rospy.init_node('testePololuSensor')

    rospy.Subscriber( "/pololuSensor", UInt8, pololu_cb)
    
    rospy.spin()
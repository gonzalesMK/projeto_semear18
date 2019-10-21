#!/usr/bin/env python
#coding: utf-8
import rospy
import smach_ros
import smach    

from std_msgs.msg import Float64, Bool
from robot_strategy.lineSensors import LineSensor, Sides
from robot_strategy.containerSensorsLib import ContainerSensors
from robot_strategy.motorControlLib import MotorControl, Wheels
from robot_strategy.utils import Colors, Positions, finalAlignmentIntersection
from robot_strategy.clawLib import Claw

import numpy as np

from robot_strategy.statesLib import strategyPlanner

container_list = [
    # Mais alto ----------------------------> Mais baixo 
    [Colors.Green, Colors.Blue, Colors.Red, Colors.Blue], # 1
    [Colors.Green, Colors.Green, Colors.Green, Colors.Red], #2    
    [Colors.Green, Colors.Green, Colors.Blue, Colors.Green], # 3 
    [Colors.Blue, Colors.Green, Colors.Blue, Colors.Green], # 4
    [Colors.Blue, Colors.Blue, Colors.Blue, Colors.Blue], #5 
    [Colors.Green, Colors.Blue, Colors.Green, Colors.Blue], # 6 
    [Colors.Green, Colors.Blue, Colors.Blue, Colors.Blue], # 7 
    [Colors.Blue, Colors.Blue, Colors.Blue, Colors.Blue], # 8  
    [], # Green Dock
    [], # Blue Dock
]

if __name__ == '__main__':
    print("Container List: \n{}".format(container_list))
    result = strategyPlanner(container_list)    
    print("Result: \n{}".format(result))

    pass
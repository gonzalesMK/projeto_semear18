#!/usr/bin/env python

import roslib; #roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros

from projeto_semear.msg import goToFirstPoseAction
from enum import Enum
# Check SMACH ROS description file to understand each state 

class Colors(Enum):
    Red = 0
    Green = 1
    Blue = 2
    Unknown = 3
    def __int__(self):
        return self.value

    def __eq__(self, other):
        return int(self) == int(other)
    
    def __ne__(self, other):
        return int(self) != int(other)

class Positions(Enum):
    StartPosition = 0
    Container1 = 1
    Container2 = 2
    Container3 = 3
    Container4 = 4
    Container5 = 5
    Container6 = 6
    Container7 = 7
    Container8 = 8
    GreenIntersection = 9
    BlueIntersection = 10
    GreenDock = 11
    BlueDock = 12
    Unkown = 13

    def __int__(self):
        return self.value

    def __eq__(self, other):
        return int(self) == int(other)
    
    def __ne__(self, other):
        return int(self) != int(other)
        
#!/usr/bin/env python

import roslib; #roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros

from enum import Enum
from robot_strategy.lineSensors import LineSensor, Sides
from robot_strategy.containerSensorsLib import ContainerSensors
from robot_strategy.motorControlLib import MotorControl, Wheels


# Check SMACH ROS description file to understand each state 
class Colors(Enum):
    Red = 0
    Green = 1
    Blue = 2
    Unknown = 3
    Empty = 4
    
    def __int__(self):
        return self.value

    def __eq__(self, other):
        return int(self) == int(other)
    
    def __ne__(self, other):
        return int(self) != int(other)

def finalAlignmentIntersection(linesensors, motorControl, rate=100):
        is_stable = 0
        r = rospy.Rate(rate)
        error = [0,0,0,0]
        
        while(not rospy.is_shutdown()):
            sensors = linesensors.readLines()
            
            error[int(Wheels.FL)] = sensors[int(Sides.LEFT)]   + sensors[int(Sides.FRONT)]
            error[int(Wheels.FR)] = sensors[int(Sides.RIGHT)]  - sensors[int(Sides.FRONT)]
            error[int(Wheels.BL)] = sensors[int(Sides.LEFT)]   - sensors[int(Sides.FRONT)]
            error[int(Wheels.BR)] = sensors[int(Sides.RIGHT)]  + sensors[int(Sides.FRONT)]

            motorControl.align(error)        
            
            if( sensors[ int(Sides.LEFT)] == 0 and sensors[int(Sides.RIGHT)] == 0 and sensors[int(Sides.FRONT)] == 0 ):
                is_stable += 1
                if( is_stable == 30):
                    break
            else:
                is_stable = 0
                        
            r.sleep()


def followLineSide(linesensors, motorControl, rate=100, coef=1):
        is_stable = 0
        r = rospy.Rate(rate)
        error = [0,0,0,0]

        while(not rospy.is_shutdown()):
            sensors = linesensors.readLines()
        
            error[int(Wheels.FL)] = sensors[int(Sides.LEFT)]  - 2   * coef
            error[int(Wheels.FR)] = sensors[int(Sides.RIGHT)] + 2   * coef
            error[int(Wheels.BL)] = sensors[int(Sides.LEFT)]  + 2  * coef
            error[int(Wheels.BR)] = sensors[int(Sides.RIGHT)] - 2  * coef
        
            if( sensors[int(Sides.FRONT)] == 0 ):
                error[int(Wheels.FL)] = sensors[int(Sides.LEFT)]
                error[int(Wheels.FR)] = sensors[int(Sides.RIGHT)]
                error[int(Wheels.BL)] = sensors[int(Sides.LEFT)]
                error[int(Wheels.BR)] = sensors[int(Sides.RIGHT)] 

                is_stable = is_stable + 1
                rospy.loginfo("is stable {}".format(is_stable))
                if is_stable == 30:
                    break

            elif( sensors[int(Sides.FRONT)] != 0 and is_stable > 0):
                is_stable = 0
                coef = coef * -1
                rospy.loginfo("coef {}".format(coef))

            motorControl.align(error)        

            r.sleep()

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
        
#! /usr/bin/env python

import rospy
from std_msgs.msg import Float64
from robot_strategy.clawLib import Claw, ServoPose
from geometry_msgs.msg import Twist
from robot_strategy.lineSensors import Sides
import numpy as np
import sys, signal
from functools import partial
"""
Interface to control the claw with the teleop_twist_keyboard:
    Linear x ( 'i' , ',') : control the Rack and Pinion
    Linear y ( 'J' , 'L') : turn On/Of the electromagnet
    Angular z ('j', 'l' ): control the Servomotor
"""
x=0
y=0
yaw=0

#clawControl=0
rack_pose = 4
servo_pose = 0

def signal_handler(clawControl, signal, frame):
    print("\nprogram exiting gracefully")
    clawControl.stop()
    print("\nprogram exiting gracefully")
    sys.exit(0)


"""
Pick
4 Containers height: 6.98670163167
3 containers height: 11.1205686748


Drop
0 container height: 18 (first container to drop)
1 container height: 14 (Second container to drop)
Servo Pose left: 5
Servo Pose Right: 165
"""

if __name__ == '__main__':
    global clawControl

    rospy.init_node('testeClawLib')
    
    clawControl = Claw()
    old_pose = clawControl.middle
    clawControl.setServoMiddle()

    signal.signal(signal.SIGINT, partial(signal_handler, clawControl), )

    while not rospy.is_shutdown():
        cm = input("Would you like to:\n" +
        "(1) Reset Claw Height\n" +
        "(2) Pick Container\n" +
        "(3) Drop Container\n" +
        "(4) Set Servo Pose\n" +
        "(5) Print height\n" + 
        "(6) Pick and Drop 4 Containers" +
        "(0)Quit\n"+
        "Command: ")

        if( cm == 1):
            clawControl.resetGearAndPinionPose()
            rospy.loginfo("New initial height: {}".format(clawControl.startPose))
        
        if( cm == 2):
            clawControl.pickContainer()

        if( cm == 3):
            height = int(input("\tHeight: "))
            clawControl.dropContainer(height)

        if( cm == 4):
            pose = int(input("\tPose: "))
            clawControl.setServoPose(initial=old_pose, end=pose)
            old_pose=pose

        if (cm == 5):
            rospy.loginfo("Hieght: {}".format(clawControl.gearAndPinionHeight))

        if (cm == 6):
            clawControl.resetGearAndPinionPose()
            clawControl.setServoPose(clawControl.middle, 160)
            clawControl.pickContainer()
            clawControl.resetGearAndPinionPose()
            clawControl.setServoPose(160, 5)
            clawControl.dropContainer(18)
            clawControl.resetGearAndPinionPose()
            clawControl.setServoPose(5, 160)
            clawControl.pickContainer()
            clawControl.resetGearAndPinionPose()
            clawControl.setServoPose(160, 5)
            clawControl.dropContainer(14)
            clawControl.resetGearAndPinionPose()
            clawControl.setServoPose(5, 160)
            clawControl.pickContainer()
            clawControl.resetGearAndPinionPose()
            clawControl.setServoPose(160, 5)
            clawControl.dropContainer(10)
            clawControl.resetGearAndPinionPose()
            clawControl.setServoPose(5, 160)
            clawControl.pickContainer()
            clawControl.resetGearAndPinionPose()
            clawControl.setServoPose(160, 5)
            clawControl.dropContainer(6)
            clawControl.resetGearAndPinionPose()

        if( cm == 0):
            print("Break")
            break

    

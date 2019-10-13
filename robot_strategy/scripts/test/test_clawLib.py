#! /usr/bin/env python

import rospy
from std_msgs.msg import Float64
from robot_strategy.clawLib import Claw, ServoPose
from geometry_msgs.msg import Twist
from robot_strategy.lineSensors import Sides
import numpy as np

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

def cmdvel_cb(msg):
    global clawControl, rack_pose, servo_pose
    
    if( msg.linear.x > 0):
        rack_pose += 4
        rospy.loginfo("Rack going up by {} cm, total height {}".format(msg.linear.x, rack_pose))
    elif( msg.linear.x < 0):
        rack_pose += -4
        rospy.loginfo("Rack going down by {} cm, total height {}".format(msg.linear.x, rack_pose))

    if( msg.angular.z > 0):     
        servo_pose += msg.angular.z
        rospy.loginfo("Servo turn clockwise by {}, total value {}".format(msg.angular.z, servo_pose ))
    elif( msg.angular.z < 0):     
        servo_pose += msg.angular.z
        rospy.loginfo("Servo turn counter-clockwise by {}, total value {}".format(msg.angular.z, servo_pose ))
    
    if( msg.linear.y > 0):
        rospy.loginfo("Turning On Electromagnet")
    if( msg.linear.y < 0):
        rospy.loginfo("Turning Off Electromagnet")

    clawControl.setGearAndPinionPose(rack_pose)    
    clawControl.setServoPose(servo_pose)

if __name__ == '__main__':
    global clawControl

    rospy.init_node('testeMotor')
    
    clawControl = Claw()

    rospy.Subscriber( "/cmd_vel", Twist, cmdvel_cb)
    
    rospy.spin()

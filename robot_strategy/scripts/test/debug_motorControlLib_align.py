#! /usr/bin/env python
import rospy
import actionlib
from std_msgs.msg import Float64, Bool
from robot_strategy.motorControlLib import MotorControl, Wheels

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
    rospy.init_node('testeMotor')
    
    motorControl = MotorControl()

    rospy.Subscriber( "/cmd_vel", Twist, cmdvel_cb)
    
    linesensors = LineSensor()
    error = [0,0,0,0]
    sensors = linesensors.readLines()

    while not rospy.is_shutdown() :
        cm = input("Would you like to:\n(1)Go Right\n(2)Go Left\n(3) Go in front ahead'til find black line\n(4)Go back'til find black line(5)Quit\nCommand: ")


        if( cm == 1 or cm==2):
            
            # A) Get the front sensor out of the black
            rospy.loginfo("Get front Sensor out of black ")
            r = rospy.Rate(100)
            if( cm == 1): 
                coef = 1
            elif (cm==2):
                coef = -1

            while(not rospy.is_shutdown()):
                sensors = linesensors.readLines()
                
                error[int(Wheels.FL)] = sensors[int(Sides.LEFT)]  - 2 * coef
                error[int(Wheels.FR)] = sensors[int(Sides.RIGHT)] + 2 * coef
                error[int(Wheels.BL)] = sensors[int(Sides.LEFT)]  + 2 * coef
                error[int(Wheels.BR)] = sensors[int(Sides.RIGHT)] - 2 * coef

                motorControl.align(error)        
                
                if( sensors[int(Sides.FRONT)] == 2 * coef ):
                    break
                            
                r.sleep()


            # B) Go to the right following the line
            rospy.loginfo("Keep going'til find next black ")
            r = rospy.Rate(100)
            is_stable = 0

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
    
            
            # C) Final Alignment
            is_stable = 0
            rospy.loginfo("Final alignment ")
            while(not rospy.is_shutdown()):
                sensors = linesensors.readLines()
                
                error[int(Wheels.FL)] = sensors[int(Sides.LEFT)]
                error[int(Wheels.FR)] = sensors[int(Sides.RIGHT)]
                error[int(Wheels.BL)] = sensors[int(Sides.LEFT)]
                error[int(Wheels.BR)] = sensors[int(Sides.RIGHT)] 

                motorControl.align(error)        
                
                if( sensors[ int(Sides.LEFT)] == 0 and sensors[int(Sides.RIGHT)] == 0 ):
                    is_stable += 1
                    if( is_stable == 30):
                        break
                else:
                    is_stable = 0
                            
                r.sleep()

            motorControl.stop()

    
        elif( cm == 5):
            motorControl.stop()
            break

    
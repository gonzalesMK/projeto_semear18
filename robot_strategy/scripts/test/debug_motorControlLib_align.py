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
    
    motorControl = MotorControl(Kp=0.0, Kd= 0.0, momentum= 0.)
    
    signal.signal(signal.SIGINT, partial(signal_handler, motorControl), )
    containerSensors = ContainerSensors()
    rospy.Subscriber( "/cmd_vel", Twist, cmdvel_cb)
    
    linesensors = LineSensor()
    error = np.array([0,0,0,0])
    sensors = linesensors.readLines()

    while not rospy.is_shutdown() :
        cm = input("Would you like to:\n(1)Go Right\n(2)Go Left\n(3) Go in front ahead'til find black line\n(4)Go back'til find black line\n(5) Go to Container \n(6)Quit\nCommand: ")
        # Frontal:      
        #      2
        #      1
        #      0
        #     -1
        #     -2

        if( cm == 1 or cm==2):
            
            # A) Get the front sensor out of the black
            rospy.loginfo("Get front Sensor out of black ")
            r = rospy.Rate(100)

            motorControl.setParams(Kp=20, Kd=100, freq=100, momentum=0.6)
            motorControl.clear()

            if( cm == 1): 
                coef = - 1
                linesensors.reset(False)
            elif (cm==2):
                coef =  1
                linesensors.reset()

            # BR positovo e FL tbm

            sensors = linesensors.readLines()
            if sensors[int(Sides.FRONT)] !=  2 * coef :    
                
                while(not rospy.is_shutdown()):
                    sensors = linesensors.readLines()
                    
                    error[int(Wheels.FL)] = - sensors[int(Sides.RIGHT)] + 2 * coef
                    error[int(Wheels.FR)] = - sensors[int(Sides.LEFT)]  + 2 * coef
                    error[int(Wheels.BL)] = + sensors[int(Sides.LEFT)]  + 2 * coef
                    error[int(Wheels.BR)] = + sensors[int(Sides.RIGHT)] + 2 * coef

                    motorControl.align(error)        
                    
                    if( sensors[int(Sides.FRONT)] == 2 * coef ):
                        break
                                
                    r.sleep()


            # B) Go to the right following the line
            rospy.loginfo("Keep going'til find next black ")
            
            # a esquerda o sensor frontal aponta -2 e a direita ele aponta 2 -> ir para direita com -2 e para esquerda com 2

            r = rospy.Rate(100)
            is_stable = 0
            if (cm == 1 ):
                linesensors.reset(False)
            else:
                linesensors.reset()

            while(not rospy.is_shutdown()):
                sensors = linesensors.readLines()
            
                error[int(Wheels.FL)] = - sensors[int(Sides.RIGHT)]  - sensors[int(Sides.FRONT)] * 2
                error[int(Wheels.FR)] = - sensors[int(Sides.LEFT)]   - sensors[int(Sides.FRONT)] * 2
                error[int(Wheels.BL)] = + sensors[int(Sides.LEFT)]   - sensors[int(Sides.FRONT)] * 2
                error[int(Wheels.BR)] = + sensors[int(Sides.RIGHT)]  - sensors[int(Sides.FRONT)] * 2

                #print("RIGHT: {}  LEFT: {} FRONT: {}".format(sensors[int(Sides.RIGHT)], sensors[int(Sides.LEFT)], sensors[int(Sides.FRONT)]))
                #print( "FL {}\tFR {}\nBL {}\tBR {}\n\n\n".format(                    - sensors[int(Sides.RIGHT)],                    - sensors[int(Sides.LEFT)],                    + sensors[int(Sides.LEFT)],                    + sensors[int(Sides.RIGHT)]) )
                
                if( sensors[int(Sides.FRONT)] == 0 and sensors[int(Sides.LEFT)] == 0 and sensors[int(Sides.RIGHT)]==0 ):
                    is_stable = is_stable + 1
                    #rospy.loginfo("is stable {}".format(is_stable))
                    
                    if is_stable == 60:
                        break

                elif( sensors[int(Sides.FRONT)] != 0 and is_stable > 0):
                    is_stable = 0

                motorControl.align(error)        

                r.sleep()
            
            motorControl.stop()

        elif (cm ==3 ):

            r = rospy.Rate(100)
            motorControl.setParams(Kp=30, Kd=100, freq=100, momentum=0.6)
            motorControl.clear()


            rospy.loginfo("2) Align with the Next Line Black")
            is_stable = 0
            linesensors.reset(False)
            while(not rospy.is_shutdown()):
                sensors = linesensors.readLines()
                
                error[int(Wheels.FL)] = - sensors[int(Sides.RIGHT)]
                error[int(Wheels.FR)] = - sensors[int(Sides.LEFT)]
                error[int(Wheels.BL)] = + sensors[int(Sides.LEFT)]
                error[int(Wheels.BR)] = + sensors[int(Sides.RIGHT)] 
                
                motorControl.align(error)        
                #print("RIGHT: {}  LEFT: {} ".format(sensors[int(Sides.RIGHT)], sensors[int(Sides.LEFT)]))
                #print( "FL {}\tFR {}\nBL {}\tBR{}\n\n\n".format(                    - sensors[int(Sides.RIGHT)],                    - sensors[int(Sides.LEFT)],                    + sensors[int(Sides.LEFT)],                    + sensors[int(Sides.RIGHT)]) )
            
                if( sensors[ int(Sides.LEFT)] == 0 and sensors[int(Sides.RIGHT)] == 0 ):
                    is_stable += 1
                    if( is_stable == 100):
                        break
                else:
                    is_stable = 0
                            
                r.sleep()

            motorControl.stop()
            
        elif( cm == 5):
             # 1) Go straight Ahead Until Back sensors find blackline
            rospy.loginfo("goToContainer: 1) Go straight Ahead Until Back sensors find blackline ")
            motorControl.setParams(Kp=30, Kd=100, freq=100, momentum=0.6)
            motorControl.clear()

            error[int(Wheels.FL)] = -1
            error[int(Wheels.FR)] = -1
            error[int(Wheels.BL)] = 1
            error[int(Wheels.BR)] = 1
            motorControl.align( error )
            
            r = rospy.Rate(100)
            
            while(not rospy.is_shutdown()):
                motorControl.align( error )
                sensors = linesensors.readLines()
                # rospy.loginfo("Front: {} BACK: {}".format(sensors[int(Sides.FRONT)], sensors[int(Sides.BACK)]))
                if( sensors[ int( Sides.BACK) ] == 0 ):
                    linesensors.reset()
                    break
                            
                r.sleep()
            motorControl.stop()

            # 2) Keep going until find container
            rospy.loginfo("goToContainer: 2) Keep going until find container ( or 3 sec)")
            
            start = rospy.get_time() 
            is_stable = 0
            motorControl.clear()
            while(not rospy.is_shutdown() and (rospy.get_time() - start ) < 3):
                sensors = linesensors.readLines()
                #rospy.loginfo("LEFT: {} RIGHT: {}".format( containerSensors.sensor[int(Sides.LEFT)], containerSensors.sensor[int(Sides.RIGHT)]))
                rospy.loginfo("{}".format(rospy.get_time() - start))
                error[int(Wheels.FL)] = + sensors[int(Sides.BACK)]   - 1
                error[int(Wheels.FR)] = + sensors[int(Sides.FRONT)]  - 1
                error[int(Wheels.BL)] = + sensors[int(Sides.BACK)]   + 1
                error[int(Wheels.BR)] = + sensors[int(Sides.FRONT)]  + 1

                if( containerSensors.sensor[int(Sides.LEFT)] and False ):
                    is_stable += 1

                    error[int(Wheels.FL)] = + sensors[int(Sides.BACK)]
                    error[int(Wheels.FR)] = + sensors[int(Sides.FRONT)]
                    error[int(Wheels.BL)] = + sensors[int(Sides.BACK)]
                    error[int(Wheels.BR)] = + sensors[int(Sides.FRONT)]
                    
                    if is_stable > 50:
                        break

                motorControl.align( error )
                r.sleep()
            # motorControl.pub_motorPWM_FL.publish(0)
            # motorControl.pub_motorPWM_FR.publish(30)
            # motorControl.pub_motorPWM_BL.publish(-30)
            # motorControl.pub_motorPWM_BR.publish(0)
            motorControl.stop()

            #motorControl.align(error)    

        elif( cm == 6):
            motorControl.stop()
            break

        motorControl.stop()
    
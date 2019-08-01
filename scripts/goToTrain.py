#! /usr/bin/env python
 
import rospy
import actionlib
from projeto_semear.msg import goToTrainAction, goToTrainFeedback, goToTrainResult
from std_msgs.msg import Float64, Bool
from projeto_semear.lineSensors import LineSensor, Sides
from projeto_semear.motorControlLib import MotorControl, Wheels
from projeto_semear.utils import Positions

class goToTrainServer(object):
    # Messages to Publish Feedback/result
    _feedback = goToTrainFeedback()
    _result = goToTrainResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, goToTrainAction, execute_cb=self.execute_cb, auto_start = False)

    def execute_cb(self, goal):
        linesensors = LineSensor()
        motorControl = MotorControl()

        motorControl.setVelocity([1,1,1,1])

        ####################################################################################################
        # 1) Get through the black line ####################################################################
        ####################################################################################################
        rospy.logdebug("1) Get Through the Black Line")
        
        r = rospy.Rate(100)
        while(not rospy.is_shutdown()):
            sensors = linesensors.readLines()

            print str(sensors[ int(Sides.RIGHT)])
            print str(sensors[ int(Sides.LEFT)])

            if( sensors[ int( Sides.LEFT) ] == 0 and sensors[ int(Sides.RIGHT) ] == 0 ):
                print ("First point")
                #linesensors.reset()
                break
                       
            r.sleep()
        ####################################################################################################
        
        
        error = [0,0,0,0]

        motorControl.setVelocity([0.5,0.5,0.5,0.5])     # TEST: Slowing the car to avoid it to lose the end of line

        ####################################################################################################
        # 2 ) Go to the left following the line until its end ##############################################
        ####################################################################################################
        rospy.logdebug("2) Go to the Left Following the Line until its end")
        is_stable = 0
        while(not rospy.is_shutdown()):
            sensors = linesensors.readLines()
            
            error[int(Wheels.FL)] = sensors[int(Sides.LEFT)]  + 2
            error[int(Wheels.FR)] = sensors[int(Sides.RIGHT)] - 2
            error[int(Wheels.BL)] = sensors[int(Sides.LEFT)]  - 2
            error[int(Wheels.BR)] = sensors[int(Sides.RIGHT)] + 2

            motorControl.align(error)        

            print str(sensors[ int(Sides.LEFT)])
            if( sensors[ int(Sides.LEFT)] == 2 or sensors[int(Sides.LEFT)] == -2 ):
                is_stable += 1
                if( is_stable == 3):
                    linesensors.reset()
                    break
            else:
                is_stable = 0
                       
            r.sleep()

        for i in range(10):        
            motorControl.stop()
            rospy.Rate(20).sleep() 
        ####################################################################################################
        

        ####################################################################################################
        # 3 ) Go to the right following the line until align with red container target #####################
        ####################################################################################################
        rospy.logdebug("3) Go to the right following the line until align with red container target")
        is_stable = 0
        while(not rospy.is_shutdown()):
            sensors = linesensors.readLines()
            
            error[int(Wheels.FL)] = sensors[int(Sides.LEFT)]  - 2
            error[int(Wheels.FR)] = sensors[int(Sides.RIGHT)] + 2
            error[int(Wheels.BL)] = sensors[int(Sides.LEFT)]  + 2
            error[int(Wheels.BR)] = sensors[int(Sides.RIGHT)] - 2

            motorControl.align(error)        

            is_stable += 1
            if( is_stable == 48):
                break
                
            print (str(sensors[ int(Sides.LEFT)]) + " | " + str(sensors[ int(Sides.RIGHT)]))
            print ("Waiting until position: " + str(is_stable))
            rospy.Rate(20).sleep()

            r.sleep()
        
        print ("I think that I reached the position")
        ####################################################################################################

        motorControl.setVelocity([1,1,1,1])
        error = [0,0,0,0]
        
        for i in range(10):
            sensors = linesensors.readLines()
            
            error[int(Wheels.FL)] = sensors[int(Sides.LEFT)] - 2
            error[int(Wheels.FR)] = sensors[int(Sides.RIGHT)] - 2
            error[int(Wheels.BL)] = sensors[int(Sides.LEFT)] - 2
            error[int(Wheels.BR)] = sensors[int(Sides.RIGHT)] - 2

            motorControl.align(error)  
        
            print (str(sensors[ int(Sides.LEFT)]) + " | " + str(sensors[ int(Sides.RIGHT)]))
            
            print ("On delay:" + str(i))
            rospy.Rate(10).sleep() 

            if( sensors[ int(Sides.LEFT)] == 2 or sensors[int(Sides.RIGHT)] == 2 ):
                print("******************************BREAK******************************")
                break

        linesensors.reset()
        error = [0,0,0,0]

        ####################################################################################################
        # 4) Align with the Next Line (Green) ##############################################################
        ####################################################################################################
        rospy.logdebug("4) Align with the Next Line Green")
        is_stable = 0
        while(not rospy.is_shutdown()):
            sensors = linesensors.readLines()
            
            error[int(Wheels.FL)] = sensors[int(Sides.LEFT)]
            error[int(Wheels.FR)] = sensors[int(Sides.RIGHT)]
            error[int(Wheels.BL)] = sensors[int(Sides.LEFT)]
            error[int(Wheels.BR)] = sensors[int(Sides.RIGHT)] 

            motorControl.align(error)        
            
            print (str(sensors[ int(Sides.LEFT)]) + " | " + str(sensors[ int(Sides.RIGHT)]))

            if( sensors[ int(Sides.LEFT)] == 0 and sensors[int(Sides.RIGHT)] == 0 ):
                is_stable += 1
                if( is_stable == 50):
                    print ("I think that I'm on the green line")
                    break
            else:
                is_stable = 0
                       
            r.sleep()

        sensors = linesensors.readLines()
            
        error[int(Wheels.FL)] = sensors[int(Sides.LEFT)] - 2
        error[int(Wheels.FR)] = sensors[int(Sides.RIGHT)] - 2
        error[int(Wheels.BL)] = sensors[int(Sides.LEFT)] - 2
        error[int(Wheels.BR)] = sensors[int(Sides.RIGHT)] - 2

        motorControl.align(error)
        
        is_stable = 0
        while( is_stable < 20):
            is_stable += 1
            rospy.Rate(20).sleep()
        
        linesensors.reset()
        error = [0,0,0,0]
        ####################################################################################################


        ####################################################################################################
        # 5) Go ahead until be near enough to the red container space ######################################
        ####################################################################################################
        rospy.logdebug("5) Go ahead until be near enough to the red container space")
        is_stable = 0
        while(not rospy.is_shutdown()):
            sensors = linesensors.readLines()
            
            error[int(Wheels.FL)] = sensors[int(Sides.LEFT)]
            error[int(Wheels.FR)] = sensors[int(Sides.RIGHT)]
            error[int(Wheels.BL)] = sensors[int(Sides.LEFT)]
            error[int(Wheels.BR)] = sensors[int(Sides.RIGHT)]

            motorControl.align(error)        

            is_stable += 1
            if(is_stable == 8):
                break
                
            print (str(sensors[ int(Sides.LEFT)]) + " | " + str(sensors[ int(Sides.RIGHT)]))
            print ("Waiting until end position: " + str(is_stable))
            rospy.Rate(20).sleep()

            r.sleep()
        
        for i in range(10):        
            motorControl.stop()
            rospy.Rate(15).sleep() 

        print ("I think that I reached the position")
        ####################################################################################################


        ####################################################################################################
        ####################################################################################################
        # ********************************* TODO - DEPOSITAR O CONTAINER ********************************* #
        ####################################################################################################
        ####################################################################################################


        ####################################################################################################
        # 6) Going back to the container Area - Get through the green line #################################
        ####################################################################################################
        linesensors.reset()
        error = [2,2,2,2]
        motorControl.setVelocity([1,1,1,1])
        print (str(sensors[ int(Sides.LEFT)]) + " | " + str(sensors[ int(Sides.RIGHT)]))

        rospy.logdebug("6) Going back to the container Area - Get through the green line")
        
        is_stable = 0
        r = rospy.Rate(100)
        while(not rospy.is_shutdown()):
            sensors = linesensors.readLines()
        
            error[int(Wheels.FL)] = sensors[int(Sides.LEFT)] + 4
            error[int(Wheels.FR)] = sensors[int(Sides.RIGHT)] + 4
            error[int(Wheels.BL)] = sensors[int(Sides.LEFT)] + 4
            error[int(Wheels.BR)] = sensors[int(Sides.RIGHT)] + 4

            motorControl.align(error) 
            
            print (str(sensors[ int(Sides.LEFT)]) + " | " + str(sensors[ int(Sides.RIGHT)]))

            if( sensors[ int( Sides.LEFT) ] == -1 and sensors[ int(Sides.RIGHT) ] == -1 ):
                is_stable += 1
                if(is_stable == 3):
                    print("***************PASSEI LINHA VERDE**************************")
                    linesensors.reset()
                    break
            else:
                is_stable = 0
                       
            r.sleep()
        
        linesensors.reset()
        error = [2,2,2,2]

        print("************************** Going to black line **************************")
        ####################################################################################################
        

        ####################################################################################################
        # 7) Going back to the container Area - Align with the Next Line (Black) ###########################
        ####################################################################################################
        rospy.logdebug("7) Going back to the container Area - Align with the Next Line (Black)")
        is_stable = 0
        while(not rospy.is_shutdown()):
            sensors = linesensors.readLines()
            
            error[int(Wheels.FL)] = sensors[int(Sides.LEFT)] + 4
            error[int(Wheels.FR)] = sensors[int(Sides.RIGHT)] + 4
            error[int(Wheels.BL)] = sensors[int(Sides.LEFT)] + 4
            error[int(Wheels.BR)] = sensors[int(Sides.RIGHT)] + 4

            motorControl.align(error)        
            
            print (str(sensors[ int(Sides.LEFT)]) + " | " + str(sensors[ int(Sides.RIGHT)]))

            if( sensors[ int(Sides.LEFT)] == 0 and sensors[int(Sides.RIGHT)] == 0 ):
                is_stable += 1
                if( is_stable == 5):
                    break
            else:
                is_stable = 0
                       
            r.sleep()
        
        print("CHEGUEI LINHA PRETA")
        ####################################################################################################


        ####################################################################################################
        # 8) Go to the right following the line until the cross ###########################################
        ####################################################################################################
        rospy.logdebug("8) Go to the right following the line until the cross")
        is_stable = 0
        while(not rospy.is_shutdown()):
            sensors = linesensors.readLines()
            
            error[int(Wheels.FL)] = sensors[int(Sides.LEFT)]  - 2
            error[int(Wheels.FR)] = sensors[int(Sides.RIGHT)] + 2
            error[int(Wheels.BL)] = sensors[int(Sides.LEFT)]  + 2
            error[int(Wheels.BR)] = sensors[int(Sides.RIGHT)] - 2

            motorControl.align(error)        

            if( sensors[ int(Sides.BACK)] == 0 ):
                is_stable += 1
                if( is_stable == 5):
                    break
            else:
                is_stable = 0

            r.sleep()
        
        print ("I think that I reached the position")
        ####################################################################################################
        ####################################################################################################

        for i in range(10):        
            motorControl.stop()
            rospy.Rate(15).sleep() 

        print "Parou motores"

        #self._result.finalPose = int(Positions.GreenIntersection)
        self._as.set_succeeded()

if __name__ == '__main__':
    rospy.init_node('goToTrainNode')
    server = goToTrainServer('goToTrain')
    
    rospy.Rate(1).sleep()  
    rospy.loginfo("Hey")
    server._as.start()
    rospy.spin()
    rospy.loginfo("Hey")
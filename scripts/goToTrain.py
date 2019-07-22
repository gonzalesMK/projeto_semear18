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

        # 1) Get through the green line
        rospy.loginfo("1) Get Through the Green Line")
        
        r = rospy.Rate(100)
        while(not rospy.is_shutdown()):
            sensors = linesensors.readLines()
            
            if( sensors[ int( Sides.LEFT) ] == 2 and sensors[ int(Sides.RIGHT) ] == 2 ):
                linesensors.reset()
                break
                       
            r.sleep()
        
        error = [0,0,0,0]
        sensors = linesensors.readLines()
        
        error[int(Wheels.FL)] = sensors[int(Sides.LEFT)]
        error[int(Wheels.FR)] = sensors[int(Sides.RIGHT)]
        error[int(Wheels.BL)] = sensors[int(Sides.LEFT)]
        error[int(Wheels.BR)] = sensors[int(Sides.RIGHT)] 

        motorControl.align(error)        
        
        # 2) Align with the Next Line (Black)
        rospy.loginfo("2) Align with the Next Line Black")
        is_stable = 0
        while(not rospy.is_shutdown()):
            sensors = linesensors.readLines()
            
            error[int(Wheels.FL)] = sensors[int(Sides.LEFT)]
            error[int(Wheels.FR)] = sensors[int(Sides.RIGHT)]
            error[int(Wheels.BL)] = sensors[int(Sides.LEFT)]
            error[int(Wheels.BR)] = sensors[int(Sides.RIGHT)] 

            motorControl.align(error)        
            
            if( sensors[ int(Sides.LEFT)] == 0 and sensors[int(Sides.RIGHT)] == 0 ):
                is_stable += 1
                if( is_stable == 100):
                    break
            else:
                is_stable = 0
                       
            r.sleep()
        

        # TODO - Conditional for decide the direction
        #
        #
        #
        # end of TODO
        
        
        # 3 ) Go to the left following the line
        rospy.loginfo("3) Go to the Left Following the Line")
        is_stable = 0
        while(not rospy.is_shutdown()):
            sensors = linesensors.readLines()
            
            error[int(Wheels.FL)] = sensors[int(Sides.LEFT)]  + 2
            error[int(Wheels.FR)] = sensors[int(Sides.RIGHT)] - 2
            error[int(Wheels.BL)] = sensors[int(Sides.LEFT)]  - 2
            error[int(Wheels.BR)] = sensors[int(Sides.RIGHT)] + 2

            motorControl.align(error)        
            
            #if( sensors[int(Sides.BACK)] == 0 ):
            #    break

            print str(sensors[ int(Sides.LEFT)])
            #if( sensors[int(Sides.LEFT)] == -1 ):
            if( sensors[ int(Sides.LEFT)] == 2 or sensors[int(Sides.LEFT)] == -2 ):
                is_stable += 1
                if( is_stable == 50):
                    print "Entrou no IF"
                    linesensors.reset()
                    break
            else:
                is_stable = 0
                       
            r.sleep()
         

        '''
        # 4) Final Alignment
        is_stable = 0
        rospy.logdebug("4) Final Alignment")
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
        '''
        
        print "1) Error: ", error
        print "1) Sensors: ", sensors

        #error = [0,0,0,0]
        sensors = linesensors.readLines()

        print "2) Error: ", error
        print "2) Sensors: ", sensors

        error[int(Wheels.FL)] = sensors[int(Sides.LEFT)] + 2
        error[int(Wheels.FR)] = sensors[int(Sides.RIGHT)] + 2
        error[int(Wheels.BL)] = sensors[int(Sides.LEFT)] + 2
        error[int(Wheels.BR)] = sensors[int(Sides.RIGHT)] + 2
        
        #error = [-2,-2,-2,-2]
        #motorControl.align(error)   
        
        print "Saiu do IF"

        print "3) Error: ", error
        print "3) Sensors: ", sensors

        for i in range(10):        
            motorControl.stop()
            rospy.Rate(10).sleep()  

        print "Parou motores"

        #self._result.finalPose = int(Positions.GreenIntersection)
        self._as.set_succeeded()

if __name__ == '__main__':
    rospy.init_node('goToTrainNode')
    server = goToTrainServer('goToTrain')
    rospy.Rate(1).sleep()  
    server._as.start()
    rospy.spin()
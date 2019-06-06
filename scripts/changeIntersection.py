#! /usr/bin/env python
import rospy
import actionlib
from projeto_semear.msg import changeIntersectionAction, changeIntersectionFeedback, changeIntersectionResult
from std_msgs.msg import Float64, Bool
from projeto_semear.lineSensors import LineSensor, Sides
from projeto_semear.containerSensorsLib import ContainerSensors
from projeto_semear.motorControlLib import MotorControl, Wheels
from projeto_semear.utils import Positions

class changeIntersectionServer(object):
    # Messages to Publish Feedback/result
    _feedback = changeIntersectionFeedback()
    _result = changeIntersectionResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, changeIntersectionAction, execute_cb=self.execute_cb, auto_start = False)
 
    def execute_cb(self, goal):
        
        linesensors = LineSensor()
        motorControl = MotorControl()
        error = [0,0,0,0]
          
        coef = 1 if goal.robotPose == Positions.GreenIntersection else -1 #check the side: should go to the right or left
         
         # 1) Get the front sensor out of the black line 
        rospy.logdebug("changeIntersection: 1)  Get out of blak line ")
        r = rospy.Rate(100)
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

        # 2) Go to the right/left following the line
        rospy.logdebug("changeIntersection: 2) Go to the Right following the line ")
        r = rospy.Rate(100)
        while(not rospy.is_shutdown()):
            sensors = linesensors.readLines()
            
            error[int(Wheels.FL)] = sensors[int(Sides.LEFT)]  - 2  * coef  
            error[int(Wheels.FR)] = sensors[int(Sides.RIGHT)] + 2  * coef  
            error[int(Wheels.BL)] = sensors[int(Sides.LEFT)]  + 2  * coef 
            error[int(Wheels.BR)] = sensors[int(Sides.RIGHT)] - 2  * coef 

            motorControl.align(error)        
            
            if( sensors[int(Sides.FRONT)] == 0 ):
                break
                       
            r.sleep()
        
        # 3) Final Alignment
        is_stable = 0
        rospy.logdebug("changeIntersection: 3) Final Alignment")
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
        
        rospy.logdebug("changeIntersection: 4) Success")
        
        self._result.robotPose = int(Positions.BlueIntersection) if goal.robotPose == Positions.GreenIntersection else int(Positions.GreenIntersection)
        self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('changeIntersection')
    server = changeIntersectionServer(rospy.get_name())
    server._as.start()
    rospy.spin()
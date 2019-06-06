#! /usr/bin/env python
import rospy
import actionlib
from projeto_semear.msg import goFromGreenToBlueIntersectionAction, goFromGreenToBlueIntersectionFeedback, goFromGreenToBlueIntersectionResult
from std_msgs.msg import Float64, Bool
from lineSensors import LineSensor, Sides
from containerSensorsLib import ContainerSensors
from motorControlLib import MotorControl, Wheels

class goFromGreenToBlueIntersectionServer(object):
    # Messages to Publish Feedback/result
    _feedback = goFromGreenToBlueIntersectionFeedback()
    _result = goFromGreenToBlueIntersectionResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, goFromGreenToBlueIntersectionAction, execute_cb=self.execute_cb, auto_start = False)
 
    def execute_cb(self, goal):
        
        linesensors = LineSensor()
        motorControl = MotorControl()
                
        error = [0,0,0,0]
        sensors = linesensors.readLines()
        
        error[int(Wheels.FL)] = sensors[int(Sides.LEFT)]
        error[int(Wheels.FR)] = sensors[int(Sides.RIGHT)]
        error[int(Wheels.BL)] = sensors[int(Sides.LEFT)]
        error[int(Wheels.BR)] = sensors[int(Sides.RIGHT)] 

        motorControl.align(error)        

         # 1) Go straight Ahead Until Back sensors find blackline
        rospy.loginfo("goFromGreenToBlueIntersection: 1) Go to the Right following the line ")
        r = rospy.Rate(100)
        while(not rospy.is_shutdown()):
            sensors = linesensors.readLines()
            
            error[int(Wheels.FL)] = sensors[int(Sides.LEFT)]  + 2
            error[int(Wheels.FR)] = sensors[int(Sides.RIGHT)] - 2
            error[int(Wheels.BL)] = sensors[int(Sides.LEFT)]  - 2
            error[int(Wheels.BR)] = sensors[int(Sides.RIGHT)] + 2

            motorControl.align(error)        
            
            if( sensors[int(Sides.FRONT)] == - 2 ):
                break
                       
            r.sleep()

        # 1) Go straight Ahead Until Back sensors find blackline
        rospy.loginfo("goFromGreenToBlueIntersection: 2) Go to the Right following the line ")
        r = rospy.Rate(100)
        while(not rospy.is_shutdown()):
            sensors = linesensors.readLines()
            
            error[int(Wheels.FL)] = sensors[int(Sides.LEFT)]  + 2
            error[int(Wheels.FR)] = sensors[int(Sides.RIGHT)] - 2
            error[int(Wheels.BL)] = sensors[int(Sides.LEFT)]  - 2
            error[int(Wheels.BR)] = sensors[int(Sides.RIGHT)] + 2

            motorControl.align(error)        
            
            if( sensors[int(Sides.FRONT)] == 0 ):
                break
                       
            r.sleep()
        
        # 2) Final Alignment
        error = [0,0,0,0]
        sensors = linesensors.readLines()
        error[int(Wheels.FL)] = sensors[int(Sides.LEFT)]
        error[int(Wheels.FR)] = sensors[int(Sides.RIGHT)]
        error[int(Wheels.BL)] = sensors[int(Sides.LEFT)]
        error[int(Wheels.BR)] = sensors[int(Sides.RIGHT)] 
        is_stable = 0
        rospy.loginfo("goFromGreenToBlueIntersection: 2) Final Alignment")
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
        
        rospy.loginfo("goFromGreenToBlueIntersection: 3) Success")
        
        self._as.set_succeeded()

if __name__ == '__main__':
    rospy.init_node('goFromGreenToBlueIntersection')
    server = goFromGreenToBlueIntersectionServer(rospy.get_name())
    server._as.start()
    rospy.spin()
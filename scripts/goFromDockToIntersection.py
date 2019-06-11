#! /usr/bin/env python
import rospy
import actionlib
from projeto_semear.msg import goFromDockToIntersectionAction, goFromDockToIntersectionFeedback, goFromDockToIntersectionResult
from std_msgs.msg import Float64, Bool
from projeto_semear.lineSensors import LineSensor, Sides
from projeto_semear.containerSensorsLib import ContainerSensors
from projeto_semear.motorControlLib import MotorControl, Wheels
from projeto_semear.utils import Colors

class goFromDockToIntersectionServer(object):
    # Messages to Publish Feedback/result
    _feedback = goFromDockToIntersectionFeedback()
    _result = goFromDockToIntersectionResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, goFromDockToIntersectionAction, execute_cb=self.execute_cb, auto_start = False)
 
    def execute_cb(self, goal):
        
        linesensors = LineSensor()
        motorControl = MotorControl()

        coef = 1 if goal.containerColor == Colors.Green else -1 #check the side: should go to the right or left

        error = [0,0,0,0]

        # 1) 
        rospy.loginfo("goFromDockToIntersection: 1) Go straight Up Until Lateral Sensors get out of line")        
        motorControl.setVelocity([1,1,1,1])
        r = rospy.Rate(100)
        while(not rospy.is_shutdown()):
            sensors = linesensors.readLines()
                 
            if( sensors[ int(Sides.LEFT)] == 2 and sensors[int(Sides.RIGHT)] == 2 ):
                    linesensors.reset()
                    break
            r.sleep()        
        

        # 2) 
        rospy.loginfo("goFromDockToIntersection: 2) Go straight Up Until LEFT/RIGHT sensors find blackline")        
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
                if( is_stable == 30):
                    linesensors.reset()
                    break
            else:
                is_stable = 0
                       
            r.sleep()

         # 3) 
        rospy.logdebug("goFromDockToIntersection: 3) Go to the right until find intersection")
        r = rospy.Rate(100)
        while(not rospy.is_shutdown()):
            sensors = linesensors.readLines()
            
            error[int(Wheels.FL)] = sensors[int(Sides.LEFT)]  - 2 * coef 
            error[int(Wheels.FR)] = sensors[int(Sides.RIGHT)] + 2 * coef
            error[int(Wheels.BL)] = sensors[int(Sides.LEFT)]  + 2 * coef
            error[int(Wheels.BR)] = sensors[int(Sides.RIGHT)] - 2 * coef

            motorControl.align(error)        
            
            if( sensors[ int(Sides.FRONT) ] ==  0 ):
                linesensors.reset()
                break
                       
            r.sleep()



        # 4)
        rospy.logdebug("goFromDockToIntersection: 4) Final Alignment")
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
                if( is_stable == 30):
                    break
            else:
                is_stable = 0
                       
            r.sleep()


        motorControl.stop()
        
        rospy.logdebug("goFromDockToIntersection: 5) Success")
        
        self._as.set_succeeded()

if __name__ == '__main__':
    rospy.init_node('goFromDockToIntersection')
    server = goFromDockToIntersectionServer(rospy.get_name())
    server._as.start()
    rospy.spin()
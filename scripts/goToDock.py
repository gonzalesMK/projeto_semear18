#! /usr/bin/env python
import rospy
import actionlib
from projeto_semear.msg import goToDockAction, goToDockFeedback, goToDockResult
from std_msgs.msg import Float64, Bool
from projeto_semear.lineSensors import LineSensor, Sides
from projeto_semear.containerSensorsLib import ContainerSensors
from projeto_semear.motorControlLib import MotorControl, Wheels
from projeto_semear.utils import Colors
class goToDockServer(object):
    # Messages to Publish Feedback/result
    _feedback = goToDockFeedback()
    _result = goToDockResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, goToDockAction, execute_cb=self.execute_cb, auto_start = False)
 
    def execute_cb(self, goal):
        
        linesensors = LineSensor()
        motorControl = MotorControl()

        coef = 1 if goal.containerColor == Colors.Green else -1 #check the side: should go to the right or left

        error = [0,0,0,0]

         # 1) Go straight Ahead Until Back sensors find blackline
        time = rospy.Time.now()
        rospy.logdebug("goToDock: 1) Go to the Right following the line {}".format(time))
        r = rospy.Rate(100)
        
        while(not rospy.is_shutdown()):
            sensors = linesensors.readLines()
            
            error[int(Wheels.FL)] = sensors[int(Sides.LEFT)]  + 2 * coef 
            error[int(Wheels.FR)] = sensors[int(Sides.RIGHT)] - 2 * coef
            error[int(Wheels.BL)] = sensors[int(Sides.LEFT)]  - 2 * coef
            error[int(Wheels.BR)] = sensors[int(Sides.RIGHT)] + 2 * coef

            motorControl.align(error)        
            
            if( rospy.Time.now() - time > rospy.Duration(3) ):
                break
                       
            r.sleep()


        # 1) Go straight Ahead Until Back sensors find blackline
        rospy.logdebug("goToDock: 2) Go DOWN until get out of black line ")
        
        motorControl.setVelocity([-1,-1,-1,-1])
        r = rospy.Rate(100)
        while(not rospy.is_shutdown()):
            sensors = linesensors.readLines()
            
            if( sensors[ int( Sides.LEFT) ] ==  - 2 and sensors[ int(Sides.RIGHT) ] == - 2 ):
                linesensors.reset(resetToMinimun=False)
                break
                       
            r.sleep()        # 2) Final Alignment
        

        rospy.logdebug("goToDock: 2) Go Down until find green line")
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
        
        rospy.logdebug("goToDock: 3) Success")
        
        self._as.set_succeeded()

if __name__ == '__main__':
    rospy.init_node('goToDock')
    server = goToDockServer(rospy.get_name())
    server._as.start()
    rospy.spin()
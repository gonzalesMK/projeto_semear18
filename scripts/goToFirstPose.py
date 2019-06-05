#! /usr/bin/env python
 
import rospy
import actionlib
from projeto_semear.msg import goToFirstPoseAction, goToFirstPoseFeedback, goToFirstPoseResult
from std_msgs.msg import Float64, Bool
from lineSensors import LineSensor, Sides
from motorControlLib import MotorControl, Wheels
from utils import Positions

class goToFirstPoseServer(object):
    # Messages to Publish Feedback/result
    _feedback = goToFirstPoseFeedback()
    _result = goToFirstPoseResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, goToFirstPoseAction, execute_cb=self.execute_cb, auto_start = False)

    def execute_cb(self, goal):
        linesensors = LineSensor()
        motorControl = MotorControl()

        rospy.loginfo("Wait")        
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
        
        # 3 ) Go to the right following the line
        rospy.loginfo("3) Go to the Right Following the Line")
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
        
        # 4) Final Alignment
        is_stable = 0
        rospy.loginfo("4) Final Alignment")
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

        self._result.final_pose.data = int(Positions.GreenIntersection)
        self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('goToFirstPoseNode')
    server = goToFirstPoseServer('goToFirstPose')
    rospy.Rate(1).sleep()  
    server._as.start()
    rospy.spin()
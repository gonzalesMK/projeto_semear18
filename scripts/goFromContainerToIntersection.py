#! /usr/bin/env python
import rospy
import actionlib
from projeto_semear.msg import goFromContainerToIntersectionAction, goFromContainerToIntersectionFeedback, goFromContainerToIntersectionResult
from std_msgs.msg import Float64, Bool
from lineSensors import LineSensor, Sides
from containerSensorsLib import ContainerSensors
from motorControlLib import MotorControl, Wheels

class goFromContainerToIntersectionServer(object):
    # Messages to Publish Feedback/result
    _feedback = goFromContainerToIntersectionFeedback()
    _result = goFromContainerToIntersectionResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, goFromContainerToIntersectionAction, execute_cb=self.execute_cb, auto_start = False)
 
    def execute_cb(self, goal):
        
        linesensors = LineSensor()
        motorControl = MotorControl()
                
        motorControl.setVelocity([-1,-1,-1,-1])

        # 1) Go straight Ahead Until Back sensors find blackline
        rospy.loginfo("goFromContainerToIntersection: 1) Go back until lateral sensors find blackline ")
        r = rospy.Rate(100)
        
        while(not rospy.is_shutdown()):
            sensors = linesensors.readLines()
            # rospy.loginfo("Front: {} BACK: {}".format(sensors[int(Sides.FRONT)], sensors[int(Sides.BACK)]))
            if( sensors[ int( Sides.LEFT) ] == 0 and sensors[ int(Sides.RIGHT) ] == 0 ):
                linesensors.reset()
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
        rospy.loginfo("goFromContainerToIntersection: 2) Final Alignment")
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
        
        rospy.loginfo("goFromContainerToIntersection: 3) Success")
        
        self._as.set_succeeded()

if __name__ == '__main__':
    rospy.init_node('goFromContainerToIntersection')
    server = goFromContainerToIntersectionServer(rospy.get_name())
    server._as.start()
    rospy.spin()
#! /usr/bin/env python
import rospy
import actionlib
from projeto_semear.msg import goToContainerAction, goToContainerFeedback, goToContainerResult
from std_msgs.msg import Float64, Bool
from lineSensors import LineSensor, Sides
from containerSensorsLib import ContainerSensors
from motorControlLib import MotorControl, Wheels

class goToContainerServer(object):
    # Messages to Publish Feedback/result
    _feedback = goToContainerFeedback()
    _result = goToContainerResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, goToContainerAction, execute_cb=self.execute_cb, auto_start = False)
 
    def execute_cb(self, goal):
        
        linesensors = LineSensor()
        motorControl = MotorControl()
        containerSensors = ContainerSensors()
        
        motorControl.setVelocity([1,1,1,1])

        # 1) Go straight Ahead Until Back sensors find blackline
        rospy.loginfo("goToContainer: 1) Go straight Ahead Until Back sensors find blackline ")
        r = rospy.Rate(100)
        
        while(not rospy.is_shutdown()):
            sensors = linesensors.readLines()
            # rospy.loginfo("Front: {} BACK: {}".format(sensors[int(Sides.FRONT)], sensors[int(Sides.BACK)]))
            if( sensors[ int( Sides.BACK) ] == 0 ):
                linesensors.reset()
                break
                       
            r.sleep()
        
        error = [0,0,0,0]
        sensors = linesensors.readLines()
        
        error[int(Wheels.FL)] = + sensors[int(Sides.FRONT)] - 2
        error[int(Wheels.FR)] = - sensors[int(Sides.FRONT)] - 2
        error[int(Wheels.BL)] = - sensors[int(Sides.BACK)]  - 2
        error[int(Wheels.BR)] = + sensors[int(Sides.BACK)]  - 2

        motorControl.align(error)        
        
        # 2) Keep going until find container
        rospy.loginfo("goToContainer: 2) Keep going until find container")

        while(not rospy.is_shutdown()):
            sensors = linesensors.readLines()
            # rospy.loginfo("LEFT: {} RIGHT: {}".format( containerSensors.sensor[int(Sides.LEFT)], containerSensors.sensor[int(Sides.RIGHT)]))

            error[int(Wheels.FL)] = + sensors[int(Sides.FRONT)] - 2
            error[int(Wheels.FR)] = - sensors[int(Sides.FRONT)] - 2
            error[int(Wheels.BL)] = - sensors[int(Sides.BACK)] - 2
            error[int(Wheels.BR)] = + sensors[int(Sides.BACK)] - 2

            if (containerSensors.sensor[int(Sides.LEFT)]):
                error[int(Wheels.FL)] = 0
                error[int(Wheels.BL)] = 0
            elif containerSensors.sensor[int(Sides.RIGHT)]:
                error[int(Wheels.FR)] = 0
                error[int(Wheels.BR)] = 0

            motorControl.align(error)        
            is_stable = 0

            if( containerSensors.sensor[int(Sides.LEFT)] and containerSensors.sensor[int(Sides.RIGHT)]):
                break

            r.sleep()
        
        # 4) Final Alignment
        is_stable = 0
        rospy.loginfo("goToContainer: 4) Final Alignment")
        while(not rospy.is_shutdown()):
            sensors = linesensors.readLines()
            # rospy.loginfo("Front: {} BACK: {} Stable: {}".format(sensors[int(Sides.FRONT)], sensors[int(Sides.BACK)], is_stable))
            error[int(Wheels.FL)] = + sensors[int(Sides.FRONT)] 
            error[int(Wheels.FR)] = - sensors[int(Sides.FRONT)] 
            error[int(Wheels.BL)] = - sensors[int(Sides.BACK)]  
            error[int(Wheels.BR)] = + sensors[int(Sides.BACK)]  

            motorControl.align(error)        
            if( sensors[int( Sides.BACK)] == 0 and sensors[int( Sides.FRONT)] == 0 ) :
                is_stable += 1
                if( is_stable == 30):
                    break
            else:
                is_stable = 0
                       
            r.sleep()

        motorControl.stop()
        rospy.loginfo("goToContainer: 5) Success")
        self._as.set_succeeded()

if __name__ == '__main__':
    rospy.init_node('goToContainer')
    server = goToContainerServer(rospy.get_name())
    server._as.start()
    rospy.spin()
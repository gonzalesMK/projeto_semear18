#! /usr/bin/env python
 
import rospy
import actionlib
from projeto_semear.msg import goToFirstPoseAction, goToFirstPoseFeedback, goToFirstPoseResult
from std_msgs.msg import Float64, Bool
from lineSensors import LineSensor, Sides

class goToFirstPoseServer(object):
    # Messages to Publish Feedback/result
    _feedback = goToFirstPoseFeedback()
    _result = goToFirstPoseResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, goToFirstPoseAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, goal):
        delay = rospy.Rate(10)
        # Go Straight ahead until find line 
        pubFL = rospy.Publisher('/motorFL/desired_vel', Float64, queue_size=10)    
        pubBL = rospy.Publisher('/motorBL/desired_vel', Float64, queue_size=10)    
        pubFR = rospy.Publisher('/motorFR/desired_vel', Float64, queue_size=10)    
        pubBR = rospy.Publisher('/motorBR/desired_vel', Float64, queue_size=10)
        pubEnable = rospy.Publisher('/encoder_enable', Bool, queue_size=10)    
        
        linesensors = LineSensor()
        delay.sleep()
        pubEnable.publish(True)
        
        pubFL.publish(1)
        pubBL.publish(1)
        pubFR.publish(1)
        pubBR.publish(1)

        r = rospy.Rate(100)
        # Get through the green line
        while(not rospy.is_shutdown()):
            rospy.loginfo( linesensors.readLines() )            
            sensors = linesensors.readLines()
            
            if( sensors[ int( Sides.LEFT) ] == 2 and sensors[ int(Sides.RIGHT) ] == 2 ):
                rospy.loginfo( "Passou da linha" )            
                pubEnable.publish(False)
                linesensors.reset()
                break
                       
            r.sleep()

        pubTarget = rospy.Publisher('/desired_pose', Float64, queue_size=10)    
        pubFL = rospy.Publisher('/motorSensorFL/error', Float64, queue_size=10)    
        pubBL = rospy.Publisher('/motorSensorBL/error', Float64, queue_size=10)    
        pubFR = rospy.Publisher('/motorSensorFR/error', Float64, queue_size=10)    
        pubBR = rospy.Publisher('/motorSensorBR/error', Float64, queue_size=10)
        pubEnable2 = rospy.Publisher('/pid_enable', Bool, queue_size=10)    
        
        delay.sleep()
        delay.sleep()
        delay.sleep()

        pubEnable2.publish(True)
        pubTarget.publish(0.0)
        
        # Align with the Next Line (Black)
        is_stable = 0
        while(not rospy.is_shutdown()):
            rospy.loginfo( linesensors.readLines() )            
            sensors = linesensors.readLines()
            
            pubFL.publish(sensors[int(Sides.LEFT)]*10.)
            pubBL.publish(sensors[int(Sides.LEFT)]*10.)
            pubFR.publish(sensors[int(Sides.RIGHT)]*10.)
            pubBR.publish(sensors[int(Sides.RIGHT)]*10.)
            
            if( sensors[ int(Sides.LEFT)] == 0 and sensors[int(Sides.RIGHT)] == 0 ):
                is_stable += 1
                if( is_stable == 100):
                    break
            else:
                is_stable = 0
                       
            r.sleep()
        
        self._as.set_succeeded()


if __name__ == '__main__':
    rospy.init_node('goToFirstPose')
    server = goToFirstPoseServer(rospy.get_name())
    rospy.spin()
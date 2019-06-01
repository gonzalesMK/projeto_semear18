#! /usr/bin/env python
 
import rospy
import actionlib
from projeto_semear.msg import goToFirstPoseAction, goToFirstPoseFeedback, goToFirstPoseResult

def goToFirstPoseAction(object):
    # Messages to Publish Feedback/result
    _feedback = goToFirstPoseFeedback()
    _result = goToFirstPoseResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, goToFirstPoseAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, goal):
        
        # Go Straight ahead until find line 
            
        pass

if __name__ == '__main__':
    rospy.init_node('goToFirstPose')
    server = goToFirstPoseAction(rospy.get_name())
    rospy.spin()
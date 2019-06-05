#!/usr/bin/env python

import roslib; #roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros

from projeto_semear.msg import goToFirstPoseAction
from projeto_semear.msg import goToContainerAction
from enum import Enum
from utils import Positions, Colors
# Check SMACH ROS description file to understand each state 

container_list = [
    # Mais alto ----------------------------> Mais baixo 
    [Colors.Green, Colors.Blue, Colors.Red, Colors.Blue], # 1
    [Colors.Green, Colors.Green, Colors.Green, Colors.Green], #2
    [Colors.Red, Colors.Green, Colors.Blue, Colors.Green], # 3 
    [Colors.Blue, Colors.Green, Colors.Blue, Colors.Green], # 4
    [Colors.Blue, Colors.Blue, Colors.Blue, Colors.Blue], #5 
    [Colors.Green, Colors.Blue, Colors.Green, Colors.Blue], # 6 
    [Colors.Green, Colors.Blue, Colors.Blue, Colors.Blue], # 7 
    [Colors.Blue, Colors.Blue, Colors.Blue, Colors.Blue], # 8  
    [], # Green Dock
    [], # Blue Dock
]
class strategyStep(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['containersUnknown', 'pick', 'changePosition','done', 'failed'],
                             input_keys=['pose'],
                             output_keys=['next_pose', 'pose'])
        
    def execute(self, userdata):
        containersAreKnown = True
        thereAreContainers = True
        
        rospy.loginfo("Strategy Step: The pose of the robot is {}".format(userdata.pose))

        # Check if the containers are known in this position
        if( not containersAreKnown ):
            return 'containersUnknown' # If they are not, go to recognizeContainers()
        
        # If they are known, choose which one is the best to pick
        if( userdata.pose.data == Positions.GreenIntersection ):
            firstColor = Colors.Green
            secondColor = Colors.Blue
            firstStackNumber = 0
        elif( userdata.pose.data == Positions.BlueIntersection ):
            firstColor = Colors.Blue
            secondColor = Colors.Green
            firstStackNumber = 4
        else:
            rospy.logerr("The strategyStep was called, but the robot position is {}".format(userdata.pose))
            return 'failed'

        for i in range(4):  # search for first Color first
            if container_list[i + firstStackNumber ][0] == firstColor :
                userdata.next_pose = Positions(i + 1) # The vector start in 0 but the stack number start from 1
                return 'pick'
        
        for i in range(4): # if no first Color container was found, look for the second Color
            if container_list[i+firstStackNumber][0] == secondColor :
                userdata.next_pose = Positions(i + 1)
                return 'pick'

        # If no container was found in this intersection, check if there are containers in the next one to be picked
        firstStackNumber = 4 if firstStackNumber == 0 else 4
        userdata.next_pose = Positions.BlueIntersection if userdata.pose == Positions.GreenIntersection else Positions.GreenIntersection

        for i in range(4):  
            if container_list[i + firstStackNumber ][0] == firstColor :
                userdata.next_pose = Positions(i + 1)
                return 'changePosition'
        
        for i in range(4):
            if container_list[i+firstStackNumber][0] == secondColor :
                userdata.next_pose = Positions()
                return 'changePosition'
        
        # No more containers
        return 'Done'

class moveToContainer(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['failed', 'success'],
                             input_keys=['nextPosition'],
                             output_keys=['newActualPosition'])
        
    def execute(self, userdata):
        
        # To-do: all the path planning and movement control (check C++ code)
        userdata.newActualPosition = userdata.nextPosition
        pass

        if(True):
            return 'success'

class moveToDock(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['failed', 'success'],
                             input_keys=['nextPosition'],
                             output_keys=['newActualPosition'])
        
    def execute(self, userdata):
        
        # To-do: all the path planning and movement control (check C++ code)
        userdata.newActualPosition = userdata.nextPosition
        pass

        if(True):
            return 'success'

class moveToStartAgainPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['failed', 'success'],
                             input_keys=[ 'nextPosition'],
                             output_keys=['newActualPosition'])
        
    def execute(self, userdata):
        
        # To-do: all the path planning and movement control (check C++ code)
        userdata.newActualPosition = userdata.nextPosition
        pass

        if(True):
            return 'success'

class moveChangePosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['failed', 'success'],
                             input_keys=[ 'nextPosition', 'robot'],
                             output_keys=['newActualPosition'])
        
    def execute(self, userdata):
        
        # To-do: all the path planning and movement control (check C++ code)
        userdata.newActualPosition = userdata.nextPosition
        pass

        if(True):
            return 'success'

class pickContainer(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['failed', 'success'],
                             output_keys=['nextPosition'])
        
    def execute(self, userdata):
        
        # To-do: all the claw control and update the container's map in the robot. Using the actual position one can find the container's height

        userdata.nextPosition = 0  # given the container's color, get the next position
        pass

        if(True):
            return 'success'

class dropContainer(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['failed', 'success'],
                             output_keys=['newActualPosition', 'nextPosition'])
        
    def execute(self, userdata):
        
        # To-do: all the claw control and update the container's map in the robot. Using the actual position one can find the container's height to drop
        
        userdata.nextPosition = 0  # given the actualPosition, one can find the nextPosition
        pass

        if(True):
            return 'success'

class recognizeContainers(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['failedRecognize', 'success'])
        
    def execute(self, userdata):
        
        # To-do: all the claw control and update the container's map in the robot. Using the actual position one can find the container's height to drop
        
        userdata.nextPosition = 0  # given the actualPosition, one can find the nextPosition
        pass

        if(True):
            return 'success'


def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['success', 'failed'])
    
    sis = smach_ros.IntrospectionServer('server_name', sm, '/IEEE_OPEN')
    sis.start()

    sm.userdata.sm_counter = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('goToFirstPose',
                smach_ros.SimpleActionState('goToFirstPose',
                                            goToFirstPoseAction,
                                            result_slots=['final_pose']
                                            ),
                transitions= {'succeeded':'strategyStep',
                              'preempted':'failed',
                              'aborted':'failed'
                              },
                remapping={'final_pose':'pose'}
        )

        smach.StateMachine.add('recognizeContainers', recognizeContainers(), 
                               transitions={'failedRecognize':'failed', 
                                            'success':'strategyStep'})

        smach.StateMachine.add('strategyStep', strategyStep(), 
                    transitions={   'containersUnknown':'recognizeContainers', 
                                    'pick':'moveToContainer',
                                    'changePosition':'moveChangeIntersection',
                                    'done':'success',
                                    'failed':'failed'},
                    remapping={'pose':'final_pose',
                            'pose': 'pose'})
        navigation_sm = smach.StateMachine(outcome=['success', 'failed'])

        with navigation_sm:
            
            smach.StateMachine.add('moveToContainer',
                    smach_ros.SimpleActionState('goToContainer',
                                                goToContainerAction), 
                    transitions= {  'succeeded':'strategyStep',
                                    'preempted':'failed',
                                    'aborted':'failed'},
                    remapping={'pose_array':'pose'})

            smach.StateMachine.add('pickContainer', pickContainer(), 
                        transitions={'failed':'failed', 
                                    'success':'moveToDock'},
                        remapping={'nextPosition':'nextPosition'})

            
            smach.StateMachine.add('moveToDock', moveToDock(), 
                               transitions={'failed':'failed', 
                                            'success':'dropContainer'},
                               remapping={'robot':'robot', 
                                          'newActualPosition':'ActualPosition',
                                          'nextPosition':'nextPosition'})

            smach.StateMachine.add('moveChangeIntersection', moveChangePosition(), 
                               transitions={'failed':'failed', 
                                            'success':'strategyStep'},
                               remapping={
                                   'pose':'pose',
                                   
                               })

        
        smach.StateMachine.add('dropContainer', dropContainer(), 
                               transitions={'failed':'failed', 
                                            'success':'moveToStartAgainPosition'},
                               remapping={'nextPosition':'nextPosition'})
        
        smach.StateMachine.add('moveToStartAgainPosition', moveToStartAgainPosition(), 
                               transitions={'failed':'failed', 
                                            'success':'strategyStep'},
                               remapping={'robot':'robot', 
                                          'newActualPosition':'ActualPosition',
                                          'nextPosition':'nextPosition'})


    # Eprojeto_semear/navigationAction.hxecute SMACH plan
    # rospy.Rate(1).sleep()  
    outcome = sm.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
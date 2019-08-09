#!/usr/bin/env python

import roslib; #roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros

from projeto_semear.msg import goToContainerAction
from projeto_semear.msg import goFromContainerToIntersectionAction
from projeto_semear.msg import changeIntersectionAction
from projeto_semear.msg import goToDockAction
from projeto_semear.msg import goFromDockToIntersectionAction

from projeto_semear.statesLib import goToFirstPose

from enum import Enum
from projeto_semear.utils import Positions, Colors

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
                             input_keys=['pose', 'containersList', 'containerColor', 'containerPose'],
                             output_keys=['pose', 'containerColor','containerPose','nextPose'])

    def execute(self, userdata):
        containersAreKnown = True
        
        rospy.Rate(1).sleep() ## Remove in real application, only for debbug 
        
        rospy.loginfo("Strategy Step: The pose of the robot is {}".format(userdata.pose))

        # Check if the containers are known in this position
        if( not containersAreKnown ):
            return 'containersUnknown' # If they are not, go to recognizeContainers()
        
        # If they are known, choose which one is the best to pick
        if( userdata.pose == Positions.GreenIntersection ):
            firstColor = int(Colors.Green)
            secondColor = int(Colors.Blue)
            firstStackNumber = 0
        elif( userdata.pose == Positions.BlueIntersection ):
            firstColor = int(Colors.Blue)
            secondColor = int(Colors.Green)
            firstStackNumber = 4
        else:
            rospy.logerr("The strategyStep was called, but the robot position is {}".format(userdata.pose))
            return 'failed'

        for i in range(4):  # search for first Color first
            if userdata.containersList[i + firstStackNumber ][0] == firstColor :
                userdata.containerColor = firstColor
                userdata.containerPose = int(Positions( i + firstStackNumber + 1))
                userdata.pose = userdata.pose
                rospy.loginfo("Strategy Step: Picked container {}".format(firstStackNumber))
                return 'pick'
        
        for i in range(4): # if no first Color container was found, look for the second Color
            if userdata.containersList[i+firstStackNumber][0] == secondColor :
                userdata.containerColor = secondColor
                userdata.containerPose = int(Positions( i + firstStackNumber + 1))
                userdata.pose = userdata.pose
                return 'pick'

        rospy.loginfo("No container was found")
        # If no container was found in this intersection, check if there are containers in the next one to be picked
        firstStackNumber = 4 if firstStackNumber == 0 else 4

        for i in range(4):  
            if userdata.containersList[i + firstStackNumber ][0] == firstColor :
                userdata.containerColor = firstColor
                userdata.containerPose = int(Positions( i + firstStackNumber + 1))
                userdata.pose = userdata.pose
                return 'changePosition'
        
        for i in range(4):
            if userdata.containersList[i+firstStackNumber][0] == secondColor :
                userdata.containerColor = secondColor
                userdata.containerPose = int(Positions( i + firstStackNumber + 1))
                userdata.pose = userdata.pose
                return 'changePosition'
        
        # No more containers
        return 'done'

class recognizeContainers(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             input_keys=['robotPose', 'containersList'],
                             output_keys=['containersList'],
                             outcomes=['failedRecognize', 'success'])
        
    def execute(self, userdata):
        
        # To-do: all the claw control and update the container's map in the robot. Using the actual position one can find the container's height to drop
        
        userdata.nextPosition = 0  # given the actualPosition, one can find the nextPosition
        pass

        if(True):
            return 'success'


class pickContainer(smach.State):
    """ This state picks and raises the container
    
        Inputs: 
            containersList: 
            containerPose:
        Output: 
            containersList: w/out the picked container
            
    """
    def __init__(self):
        smach.State.__init__(self, 
                            input_keys=['containersList','containerPose',],
                            output_keys=['containersList'],
                            outcomes=['preempted', 'succeeded','aborted'])
    
    def execute(self, userdata):
        color = userdata.containersList[userdata.containerPose-1].pop(0)
        
        if not userdata.containersList[userdata.containerPose-1] :
            userdata.containersList[userdata.containerPose-1].append(Colors.Empty)

        if color == Colors.Green:
            userdata.containersList[int(Positions.GreenDock)-3].append(color) 
        elif color == Colors.Blue:
            userdata.containersList[int(Positions.BlueDock)-3].append(color) 
        else:
            rospy.logerr("The color to pick the container is wrong")
        
        return 'succeeded'

class whereToGo(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            input_keys=['robotPose','containerColor'],
                            output_keys=[],
                            outcomes=['intersection', 'dock'])
    
    def execute(self, userdata):
        rospy.Rate(1).sleep() ## Remove in real application, only for debbug 
        rospy.loginfo("Pose: {} Color: {}".format(userdata.robotPose, userdata.containerColor))

        if (userdata.robotPose == Positions.GreenIntersection and userdata.containerColor == Colors.Green) or \
           (userdata.robotPose == Positions.BlueIntersection and userdata.containerColor == Colors.Blue):
            return 'dock'
        
        return 'intersection'


def main():
    rospy.init_node('smach_example_state_machine')
    rospy.loginfo("Hey")
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])
    
    sis = smach_ros.IntrospectionServer('server1', sm, '/IEEE_OPEN')
    sis.start()

    sm.userdata.containersList = container_list
    sm.userdata.containerColor = int(Colors.Unknown)
    sm.userdata.containerPose  = int(Positions.Unkown)
    sm.userdata.robotPose      = int(Positions.StartPosition)

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('goToFirstPose', goToFirstPose(),
                transitions={ 'succeeded':'strategyStep',
                              'aborted':'failed'
                            },
                remapping={'finalPose':'robotPose'}
        )

        smach.StateMachine.add('recognizeContainers', recognizeContainers(), 
                transitions={
                    'failedRecognize':'failed', 
                    'success':'strategyStep'
                    },
                remapping={
                    'finalPose':'robotPose'
                    }
        )

        smach.StateMachine.add('strategyStep', strategyStep(), 
                    transitions={   'containersUnknown':'recognizeContainers', 
                                    'pick':'pickAndDrop',
                                    'changePosition':'changeIntersection',
                                    'done':'succeeded',
                                    'failed':'failed'},
                    remapping={ 
                        'pose':'robotPose',
                        'containersList':'containersList',
                        'containerColor':'containertColor',
                        'containerPose':'containerPose',
                    }
        )

        smach.StateMachine.add('changeIntersection',
                smach_ros.SimpleActionState('changeIntersection',
                                            changeIntersectionAction,
                                            goal_slots=['robotPose'],
                                            result_slots=['robotPose']
                                            ), 
                transitions= {  'succeeded':'strategyStep',
                                'preempted':'failed',
                                'aborted':'failed'},
                remapping={ 'robotPose':'robotPose'}
        )

        pick_and_drop_sm = smach.StateMachine(
            outcomes=['succeeded', 'failed'],
            input_keys=['robotPose','containersList','containerColor','containerPose'],
            output_keys=['robotPose','containersList','containerColor','containerPose'])

        sis2 = smach_ros.IntrospectionServer('server2', pick_and_drop_sm, '/IEEE_OPEN/pickAndDrop')
        sis2.start()
        with pick_and_drop_sm:

            smach.StateMachine.add('goToContainer',
                    smach_ros.SimpleActionState('goToContainer',
                                                goToContainerAction,
                                                goal_slots=['containerPose']),
                    transitions= {  'succeeded':'pickContainer',
                                    'preempted':'failed',
                                    'aborted':'failed'},
                    remapping={ 'containerPose':'containerPose'}
            )

            smach.StateMachine.add('pickContainer', 
                    pickContainer(), 
                    transitions= {  'succeeded':'goFromContainerToIntersection',
                                    'preempted':'failed',
                                    'aborted':'failed'},
                    remapping={
                            'containersList':'containersList',
                            'containerPose':'containerPose',
                        }
            )

            smach.StateMachine.add('goFromContainerToIntersection', 
                    smach_ros.SimpleActionState('goFromContainerToIntersection',
                                                goFromContainerToIntersectionAction), 
                    transitions= {  'succeeded':'whereToGo',
                                    'preempted':'failed',
                                    'aborted':'failed'},
            )

            smach.StateMachine.add('whereToGo', 
                whereToGo(), 
                transitions={
                    'dock': 'goToDock',
                    'intersection':'changeIntersection'},
                remapping={
                    'robotPose':'robotPose',
                    'containerColor':'containerColor'}
            )
            
            smach.StateMachine.add('goToDock', 
                    smach_ros.SimpleActionState('goToDock',
                                                goToDockAction,
                                                goal_slots=['containerColor']), 
                    transitions= {  'succeeded':'goFromDockToIntersection',
                                    'preempted':'failed',
                                    'aborted':'failed'},
                    remapping={ 'containerColor':'containerColor'}
            )   

            smach.StateMachine.add('changeIntersection',
                smach_ros.SimpleActionState('changeIntersection',
                                            changeIntersectionAction,
                                            goal_slots=['robotPose'],
                                            result_slots=['robotPose']
                                            ), 
                transitions= {  'succeeded':'goToDock',
                                'preempted':'failed',
                                'aborted':'failed'},
                remapping={ 'robotPose':'robotPose'}
            )
            
            smach.StateMachine.add('goFromDockToIntersection', 
                    smach_ros.SimpleActionState('goFromDockToIntersection',
                                                goFromDockToIntersectionAction,
                                                goal_slots=['containerColor']), 
                    transitions= {  'succeeded':'succeeded',
                                    'preempted':'failed',
                                    'aborted':'failed'},
                    remapping={ 'containerColor':'containerColor'}
            )   

        smach.StateMachine.add("pickAndDrop", pick_and_drop_sm,
                    transitions={
                        'succeeded':'strategyStep',
                        'failed':'failed'
                    },
                    remapping={
                        'robotPose':'robotPose',
                        'containersList':'containersList',
                        'containerColor':'containertColor',
                        'containerPose':'containerPose',
                    })

    outcome = sm.execute()

    rospy.spin()
    sis2.stop()
    sis.stop()


if __name__ == '__main__':
    main()
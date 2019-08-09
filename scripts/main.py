#!/usr/bin/env python

import roslib; #roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros

from projeto_semear.statesLib import goToFirstPose
from projeto_semear.statesLib import goToContainer
from projeto_semear.statesLib import goFromContainerToIntersection
from projeto_semear.statesLib import changeIntersection
from projeto_semear.statesLib import goFromDockToIntersection
from projeto_semear.statesLib import goToDock
from projeto_semear.statesLib import strategyStep
from projeto_semear.statesLib import recognizeContainers
from projeto_semear.statesLib import pickContainer
from projeto_semear.statesLib import whereToGo

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

def main():
    rospy.init_node('smach_example_state_machine')

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

        smach.StateMachine.add('changeIntersection', changeIntersection(),
                transitions= {  'succeeded':'strategyStep',
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

            smach.StateMachine.add('goToContainer', goToContainer(),
                    transitions= {  'succeeded':'pickContainer',
                                    'aborted':'failed'},
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

            smach.StateMachine.add('goFromContainerToIntersection', goFromContainerToIntersection(), 
                    transitions= {  'succeeded':'whereToGo',
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
            
            smach.StateMachine.add('goToDock', goToDock(),
                    transitions= {  'succeeded':'goFromDockToIntersection',
                                    'aborted':'failed'},
                    remapping={ 'containerColor':'containerColor'}
            )   

            smach.StateMachine.add('changeIntersection', changeIntersection(),
            transitions= {  'succeeded':'goToDock',
                            'aborted':'failed'},
            remapping={ 'robotPose':'robotPose'}
                    )
            
            smach.StateMachine.add('goFromDockToIntersection', goFromDockToIntersection(),
                    transitions= {  'succeeded':'succeeded',
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
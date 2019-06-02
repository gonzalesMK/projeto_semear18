#!/usr/bin/env python

import roslib; #roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros

from projeto_semear.msg import goToFirstPoseAction
# Check SMACH ROS description file to understand each state 

class start(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['failed','success'],
                             output_keys=['robot'])
    def execute(self, userdata):
        
        # Initialize all sensors and variables
        pass

        return 'success'

class firstAlignment(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['failed','success'],
                             input_keys=['robot'])

    def execute(self, userdata):
        rospy.loginfo('Executing First Alignment')
        
        # Go straight ahead until find green line 
        pass

        #  Get Through green line
        pass

        # Go straight ahead until find black line
        pass

        # Align with black line
        pass

        if( True ):
            return 'success'
        else: 
            return 'failed'


class strategyStep(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['containersUnknown', 'pick', 'noContainers','done'],
                             input_keys=['actualPosition'],
                             output_keys=['nextPosition',])
        
    def execute(self, userdata):
        containersAreKnown = True
        thereAreContainers = True
        
        # Check if the containers are known in this position
        if( containersAreKnown == False ):
            return 'containersUnknown'
        
        # If they are known, choose which one is the best to pick
        userdata.nextPosition = 0 # 
        if (containersAreKnown == True and thereAreContainers == True):
            return 'pick'
        
        # Otherwise, change position      
        userdata.nextPosition = 0 # 
        return 'noContainers'
        

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
        smach.StateMachine.add('firstOne',
                               smach_ros.SimpleActionState('goToFirstPose', goToFirstPoseAction),
                               transitions= {'succeeded':'firstAlignment',
                                             'preempted':'failed',
                                             'aborted':'failed'})

        smach.StateMachine.add('recognizeContainers', recognizeContainers(), 
                               transitions={'failedRecognize':'failed', 
                                            'success':'strategyStep'})

        smach.StateMachine.add('firstAlignment', firstAlignment(), 
                               transitions={'success':'strategyStep',
                                            'failed': 'failed'},
                               remapping={'robot':'robot'})
                                          
        
        smach.StateMachine.add('strategyStep', strategyStep(), 
                               transitions={'containersUnknown':'recognizeContainers', 
                                            'pick':'moveToContainer',
                                            'noContainers':'moveChangePosition',
                                            'done':'success'},
                               remapping={'robot':'robot', 
                                          'nextPosition':'nextPosition'})

        smach.StateMachine.add('moveChangePosition', moveChangePosition(), 
                               transitions={'failed':'failed', 
                                            'success':'strategyStep'},
                               remapping={'robot':'robot', 
                                          'newActualPosition':'ActualPosition',
                                          'nextPosition':'nextPosition'})

        smach.StateMachine.add('moveToContainer', moveToContainer(), 
                               transitions={'failed':'failed', 
                                            'success':'pickContainer'},
                               remapping={'robot':'robot', 
                                          'newActualPosition':'ActualPosition',
                                          'nextPosition':'nextPosition'})

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
    outcome = sm.execute()


    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
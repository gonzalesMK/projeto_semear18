#!/usr/bin/env python
import rospy
import smach_ros
import smach    

from std_msgs.msg import Float64, Bool
from robot_strategy.lineSensors import LineSensor, Sides
from robot_strategy.containerSensorsLib import ContainerSensors
from robot_strategy.motorControlLib import MotorControl, Wheels
from robot_strategy.utils import Colors, Positions

class changeIntersection(smach.State):

    def __init__(self):
            smach.State.__init__(self, 
                outcomes=['succeeded', 'aborted'],
                input_keys=['robotPose'],
                output_keys=['robotPose'])

    def execute(self, userdata):
        
        linesensors = LineSensor()
        motorControl = MotorControl()
        error = [0,0,0,0]
            
        coef = 1 if userdata.robotPose == Positions.GreenIntersection else -1 #check the side: should go to the right or left
            
            # 1) Get the front sensor out of the black line 
        rospy.logdebug("changeIntersection: 1)  Get out of blak line ")
        r = rospy.Rate(100)
        while(not rospy.is_shutdown()):
            sensors = linesensors.readLines()
            
            error[int(Wheels.FL)] = sensors[int(Sides.LEFT)]  - 2 * coef
            error[int(Wheels.FR)] = sensors[int(Sides.RIGHT)] + 2 * coef
            error[int(Wheels.BL)] = sensors[int(Sides.LEFT)]  + 2 * coef
            error[int(Wheels.BR)] = sensors[int(Sides.RIGHT)] - 2 * coef

            motorControl.align(error)        
            
            if( sensors[int(Sides.FRONT)] == 2 * coef ):
                break
                        
            r.sleep()

        # 2) Go to the right/left following the line
        rospy.logdebug("changeIntersection: 2) Go to the Right following the line ")
        r = rospy.Rate(100)
        while(not rospy.is_shutdown()):
            sensors = linesensors.readLines()
            
            error[int(Wheels.FL)] = sensors[int(Sides.LEFT)]  - 2  * coef  
            error[int(Wheels.FR)] = sensors[int(Sides.RIGHT)] + 2  * coef  
            error[int(Wheels.BL)] = sensors[int(Sides.LEFT)]  + 2  * coef 
            error[int(Wheels.BR)] = sensors[int(Sides.RIGHT)] - 2  * coef 

            motorControl.align(error)        
            
            if( sensors[int(Sides.FRONT)] == 0 ):
                break
                        
            r.sleep()
        
        # 3) Final Alignment
        is_stable = 0
        rospy.logdebug("changeIntersection: 3) Final Alignment")
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
        
        userdata.robotPose = int(Positions.BlueIntersection) if userdata.robotPose == Positions.GreenIntersection else int(Positions.GreenIntersection)

        rospy.loginfo("changeIntersection: 4) Success. New pose is: {}".format(userdata.robotPose))

        return 'succeeded'

class goFromContainerToIntersection(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded', 'aborted'],
            input_keys=[],
            output_keys=[])

    def execute(self, userdata):
    
        linesensors = LineSensor()
        motorControl = MotorControl()
                
        motorControl.setVelocity([-1,-1,-1,-1])

        # 1) Go straight Ahead Until Back sensors find blackline
        rospy.logdebug("goFromContainerToIntersection: 1) Go back until lateral sensors find blackline ")
        r = rospy.Rate(100)
        
        while(not rospy.is_shutdown()):
            sensors = linesensors.readLines()
            # rospy.logdebug("Front: {} BACK: {}".format(sensors[int(Sides.FRONT)], sensors[int(Sides.BACK)]))
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
        rospy.logdebug("goFromContainerToIntersection: 2) Final Alignment")
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
        
        rospy.logdebug("goFromContainerToIntersection: 3) Success")
        
        #TODO: if this code takes to long to complete, cancel it bcs smt went wrong
        return 'succeeded'

class goFromDockToIntersection(smach.State):
    def __init__(self):
            smach.State.__init__(self, 
            outcomes=['succeeded', 'aborted'],
            input_keys=['containerColor'],
            output_keys=[])


    def execute(self, userdata):
        
        linesensors = LineSensor()
        motorControl = MotorControl()

        coef = 1 if userdata.containerColor == Colors.Green else -1 #check the side: should go to the right or left

        error = [0,0,0,0]

        # 1) 
        rospy.loginfo("goFromDockToIntersection: 1) Go straight Up Until Lateral Sensors get out of line")        
        motorControl.setVelocity([1,1,1,1])
        r = rospy.Rate(100)
        while(not rospy.is_shutdown()):
            sensors = linesensors.readLines()
                    
            if( sensors[ int(Sides.LEFT)] == 2 and sensors[int(Sides.RIGHT)] == 2 ):
                    linesensors.reset()
                    break
            r.sleep()        
        

        # 2) 
        rospy.loginfo("goFromDockToIntersection: 2) Go straight Up Until LEFT/RIGHT sensors find blackline")        
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
                    linesensors.reset()
                    break
            else:
                is_stable = 0
                        
            r.sleep()

            # 3) 
        rospy.logdebug("goFromDockToIntersection: 3) Go to the right until find intersection")
        r = rospy.Rate(100)
        while(not rospy.is_shutdown()):
            sensors = linesensors.readLines()
            
            error[int(Wheels.FL)] = sensors[int(Sides.LEFT)]  - 2 * coef 
            error[int(Wheels.FR)] = sensors[int(Sides.RIGHT)] + 2 * coef
            error[int(Wheels.BL)] = sensors[int(Sides.LEFT)]  + 2 * coef
            error[int(Wheels.BR)] = sensors[int(Sides.RIGHT)] - 2 * coef

            motorControl.align(error)        
            
            if( sensors[ int(Sides.FRONT) ] ==  0 ):
                linesensors.reset()
                break
                        
            r.sleep()

        # 4)
        rospy.logdebug("goFromDockToIntersection: 4) Final Alignment")
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
        
        rospy.logdebug("goFromDockToIntersection: 5) Success")
        
        return 'succeeded'

class goToContainer(smach.State):

    def __init__(self):
            smach.State.__init__(self, 
                outcomes=['succeeded', 'aborted'],
                input_keys=[],
                output_keys=[])
    
    
    def execute(self, userdata):
        #TODO: the robot needs to get to the first row container and the second row of container
        linesensors = LineSensor()
        motorControl = MotorControl()
        containerSensors = ContainerSensors()
        
        motorControl.setVelocity([1,1,1,1])

        # 1) Go straight Ahead Until Back sensors find blackline
        rospy.logdebug("goToContainer: 1) Go straight Ahead Until Back sensors find blackline ")
        r = rospy.Rate(100)
        
        while(not rospy.is_shutdown()):
            sensors = linesensors.readLines()
            # rospy.logdebug("Front: {} BACK: {}".format(sensors[int(Sides.FRONT)], sensors[int(Sides.BACK)]))
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
        rospy.logdebug("goToContainer: 2) Keep going until find container")

        while(not rospy.is_shutdown()):
            sensors = linesensors.readLines()
            # rospy.logdebug("LEFT: {} RIGHT: {}".format( containerSensors.sensor[int(Sides.LEFT)], containerSensors.sensor[int(Sides.RIGHT)]))

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
        rospy.logdebug("goToContainer: 4) Final Alignment")
        while(not rospy.is_shutdown()):
            sensors = linesensors.readLines()
            # rospy.logdebug("Front: {} BACK: {} Stable: {}".format(sensors[int(Sides.FRONT)], sensors[int(Sides.BACK)], is_stable))
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
        rospy.logdebug("goToContainer: 5) Success")
        
        return 'succeeded'

class goToDock(smach.State):

    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded', 'aborted'],
            input_keys=['containerColor'],
            output_keys=[])


    def execute(self, userdata):
        
        linesensors = LineSensor()
        motorControl = MotorControl()

        coef = 1 if userdata.containerColor == Colors.Green else -1 #check the side: should go to the right or left

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
        
        return "succeeded"

class goToFirstPose(smach.State):

    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded', 'aborted'],
            input_keys=[],
            output_keys=['finalPose'])

        
    def execute(self, userdata):
        linesensors = LineSensor()
        motorControl = MotorControl()

        motorControl.setVelocity([1,1,1,1])

        # 1) Get through the green line
        rospy.logdebug("1) Get Through the Green Line")
        
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
        rospy.logdebug("2) Align with the Next Line Black")
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
        
        # 3 ) Go to the left following the line
        rospy.logdebug("3) Go to the Left Following the Line")
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
        rospy.logdebug("4) Final Alignment")
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

        userdata.finalPose = int(Positions.GreenIntersection)
        return "succeeded"

################################################
################################################
##### THOSE CLASSES BELOW ARE INCOMPLETED! #####
################################################
################################################

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
# 
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

#!/usr/bin/env python
import rospy
import smach_ros
import smach    

from std_msgs.msg import Float64, Bool
from robot_strategy.lineSensors import LineSensor, Sides
from robot_strategy.containerSensorsLib import ContainerSensors
from robot_strategy.motorControlLib import MotorControl, Wheels
from robot_strategy.utils import Colors, Positions, finalAlignmentIntersection
from robot_strategy.clawLib import Claw

import numpy as np

class changeIntersection(smach.State):
    """
        The robot changes from the Green Intersection to the Blue Intersection and vice versa

        Input: robotPose - is the robot actual position

        Output: robotPose - is the robot new position after the change
    """
    def __init__(self):
            smach.State.__init__(self, 
                outcomes=['succeeded', 'aborted'],
                input_keys=['robotPose'],
                output_keys=['robotPose'])

    def execute(self, userdata):
        
        linesensors = LineSensor()
        motorControl = MotorControl()
        motorControl.setParams(Kp=30, Kd=100, freq=100, momentum=0.6)
        error = np.array([0,0,0,0])
            
        coef = -1 if userdata.robotPose == Positions.GreenIntersection else 1 #check the side: should go to the right or left
            
        # 1) Get the front sensor out of the black line 
        rospy.loginfo("changeIntersection: 1)  Get out of blak line ")
        
        motorControl.setParams(Kp=20, Kd=100, freq=100, momentum=0.6)

        r = rospy.Rate(100)
        while (not rospy.is_shutdown() and 
              (sensors[int(Sides.FRONT)] !=  2 * coef ) ) :
            
            sensors = linesensors.readLines()
            
            angular = sensors[int(Sides.LEFT)] - sensors[int(Sides.RIGHT)]   
            diretional = (sensors[int(Sides.LEFT)] + sensors[int(Sides.RIGHT)] ) / 2

            error[int(Wheels.FL)] = - diretional  + angular + coef * 1.5
            error[int(Wheels.FR)] = + diretional  - angular + coef * 1.5
            error[int(Wheels.BL)] = + diretional  + angular + coef * 1.5
            error[int(Wheels.BR)] = - diretional  - angular + coef * 1.5

            motorControl.align(error)        

            r.sleep()

        # 2) Go to the right/left following the line
        rospy.loginfo("changeIntersection: 2) Keep going'til find next black ")

        r = rospy.Rate(100)
        is_stable = 0
        
        if (coef == - 1 ):
            linesensors.reset(False)
        else:
            linesensors.reset()

        while(not rospy.is_shutdown()):
            sensors = linesensors.readLines()
        
            angular = sensors[int(Sides.LEFT)] - sensors[int(Sides.RIGHT)]   

            diretional = (sensors[int(Sides.LEFT)] + sensors[int(Sides.RIGHT)] ) / 2

            error[int(Wheels.FL)] = - diretional  + angular - sensors[int(Sides.FRONT)] * 1
            error[int(Wheels.FR)] = + diretional  - angular - sensors[int(Sides.FRONT)] * 1
            error[int(Wheels.BL)] = + diretional  + angular - sensors[int(Sides.FRONT)] * 1
            error[int(Wheels.BR)] = - diretional  - angular - sensors[int(Sides.FRONT)] * 1
            
            if( sensors[int(Sides.FRONT)] == 0 and sensors[int(Sides.LEFT)] == 0 and sensors[int(Sides.RIGHT)]==0 ):
                is_stable = is_stable + 1

                if is_stable == 60:
                    break

            elif( sensors[int(Sides.FRONT)] != 0 and is_stable > 0):
                is_stable = 0

            motorControl.align(error)        

            r.sleep()

        motorControl.stop()
        
        userdata.robotPose = int(Positions.BlueIntersection) if userdata.robotPose == Positions.GreenIntersection else int(Positions.GreenIntersection)

        rospy.loginfo("changeIntersection: 3) Success. New pose is: {}".format(userdata.robotPose))

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
        motorControl.setParams(Kp=30, Kd=100, freq=100, momentum=0.6)                
        # 1) Go straight Ahead Until Back sensors find blackline
        # rospy.loginfo("goFromContainerToIntersection: 1) Go back until lateral sensors find blackline ")
        # r = rospy.Rate(100)
        
        # while(not rospy.is_shutdown()):
        #     sensors = linesensors.readLines()
        #     # rospy.loginfo("Front: {} BACK: {}".format(sensors[int(Sides.FRONT)], sensors[int(Sides.BACK)]))
        #     if( sensors[ int( Sides.LEFT) ] == 0 and sensors[ int(Sides.RIGHT) ] == 0 ):
        #         linesensors.reset()
        #         break
        #     r.sleep()
        
        # 2) Final Alignment
        rospy.loginfo("goFromContainerToIntersection: 1) Go Back until lateral sensors find blackline")
        
        linesensors.reset(False)
        finalAlignmentIntersection(linesensors,motorControl)

        rospy.loginfo("goFromContainerToIntersection: 3) Success")
        motorControl.stop()
        
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
        motorControl.setParams(Kp=30, Kd=100, freq=100, momentum=0.6)

        coef = -1 if userdata.containerColor == Colors.Green else 1 #check the side: should go to the right or left

        error = np.array([0,0,0,0])

        # 1) 
        rospy.loginfo("goFromDockToIntersection: 1) Go straight Up Until Lateral Sensors get out of line")        
        
        linesensors.reset(False)
        error = np.array([-2,+2,+2,-2])

        r = rospy.Rate(100)
        while(not rospy.is_shutdown()):

            sensors = linesensors.readLines()
            motorControl.align(error)            
    
            if( sensors[ int( Sides.LEFT) ] == - 2 and sensors[ int(Sides.RIGHT) ] == - 2 ):
                break
                        
            r.sleep()
  
        # 2) 
        rospy.loginfo("goFromDockToIntersection: 2) Go straight Up Until LEFT/RIGHT sensors find blackline")        

        linesensors.reset(False)
        is_stable = 0
        while(not rospy.is_shutdown()):
            sensors = linesensors.readLines()
            
            angular = sensors[int(Sides.LEFT)] - sensors[int(Sides.RIGHT)]   
            diretional = (sensors[int(Sides.LEFT)] + sensors[int(Sides.RIGHT)] ) / 2

            error[int(Wheels.FL)] = - diretional  + angular 
            error[int(Wheels.FR)] = + diretional  - angular 
            error[int(Wheels.BL)] = + diretional  + angular 
            error[int(Wheels.BR)] = - diretional  - angular 
            
            motorControl.align(error)        
        
            if( sensors[ int(Sides.LEFT)] == 0 and sensors[int(Sides.RIGHT)] == 0 ):
                is_stable += 1
                if( is_stable == 100):
                    break
            else:
                is_stable = 0
                        
            r.sleep()

        motorControl.stop() 
        
        # 3) 
        rospy.loginfo("goFromDockToIntersection: 3) Go to the side until find intersection")
        r = rospy.Rate(100)
        while (not rospy.is_shutdown() and 
              (sensors[int(Sides.FRONT)] !=  2 * coef ) ) :
            
            sensors = linesensors.readLines()
            
            angular = sensors[int(Sides.LEFT)] - sensors[int(Sides.RIGHT)]   
            diretional = (sensors[int(Sides.LEFT)] + sensors[int(Sides.RIGHT)] ) / 2

            error[int(Wheels.FL)] = - diretional  + angular + coef * 1.5
            error[int(Wheels.FR)] = + diretional  - angular + coef * 1.5
            error[int(Wheels.BL)] = + diretional  + angular + coef * 1.5
            error[int(Wheels.BR)] = - diretional  - angular + coef * 1.5

            motorControl.align(error)        

            r.sleep()

        motorControl.stop()
        
        rospy.loginfo("goFromDockToIntersection: 4) Success")
        
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
        rospy.loginfo("goToContainer: 1) Go straight Ahead Until Back sensors find blackline ")
        motorControl.setParams(Kp=30, Kd=100, freq=100, momentum=0.6)
        motorControl.clear()

        error[int(Wheels.FL)] = - 1
        error[int(Wheels.FR)] = + 1
        error[int(Wheels.BL)] = + 1
        error[int(Wheels.BR)] = - 1
        motorControl.align( error )
        
        r = rospy.Rate(100)
        while(not rospy.is_shutdown()):
            motorControl.align( error )
            sensors = linesensors.readLines()
            # rospy.loginfo("Front: {} BACK: {}".format(sensors[int(Sides.FRONT)], sensors[int(Sides.BACK)]))
            if( sensors[ int( Sides.BACK) ] == 0 ):
                linesensors.reset()
                break
                        
            r.sleep()
        motorControl.stop()
        # 2) Keep going until find container
        rospy.loginfo("goToContainer: 2) Keep going until find container")
        rospy.loginfo("goToContainer: 2) Keep going until find container ( or 3 sec)")
        
        start = rospy.get_time() 
        is_stable = 0
        motorControl.clear()
        while(not rospy.is_shutdown() and (rospy.get_time() - start ) >0):
            sensors = linesensors.readLines()
            #rospy.loginfo("LEFT: {} RIGHT: {}".format( containerSensors.sensor[int(Sides.LEFT)], containerSensors.sensor[int(Sides.RIGHT)]))
            #rospy.loginfo("{}".format(rospy.get_time() - start))

            print("RIGHT: {}  LEFT: {} FRONT: {}".format(sensors[int(Sides.RIGHT)], sensors[int(Sides.LEFT)], sensors[int(Sides.FRONT)]))
            print( "FL {}\tFR {}\nBL {}\tBR {}\n\n\n".format(error[int(Wheels.FL)],                   
                    error[int(Wheels.FR)],  error[int(Wheels.BL)], error[int(Wheels.BR)]) )

            angular = sensors[int(Sides.FRONT)] - sensors[int(Sides.BACK)]   

            diretional = (sensors[int(Sides.FRONT)] + sensors[int(Sides.BACK)] ) / 2

            error[int(Wheels.FL)] = - diretional  + angular  - 1
            error[int(Wheels.FR)] = - diretional  - angular  + 1
            error[int(Wheels.BL)] = - diretional  + angular  + 1
            error[int(Wheels.BR)] = - diretional  - angular  - 1
            print()
            if( containerSensors.sensor[int(0)] and False ):
                is_stable += 1

                error[int(Wheels.FL)] = + sensors[int(Sides.BACK)]
                error[int(Wheels.FR)] = + sensors[int(Sides.FRONT)]
                error[int(Wheels.BL)] = + sensors[int(Sides.BACK)]
                error[int(Wheels.BR)] = + sensors[int(Sides.FRONT)]
                
                if is_stable > 50:
                    break

            motorControl.align( error )
            r.sleep()

        motorControl.stop()

        rospy.loginfo("goToContainer: 5) Success")
        
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
        motorControl.setParams(Kp=30, Kd=100, freq=100, momentum=0.6)
        coef = -1 if userdata.containerColor == Colors.Green else 1 #check the side: should go to the right or left

        error = np.array([0,0,0,0])

        # 1) 
        time = rospy.Time.now()
        rospy.loginfo("goToDock: 1) Go to the Side following the line during {}s".format(time))
        r = rospy.Rate(100)
        while(not rospy.is_shutdown() and 
            ( rospy.Time.now() - time < rospy.Duration(3) ) ):
            
            sensors = linesensors.readLines()
            
            angular = sensors[int(Sides.LEFT)] - sensors[int(Sides.RIGHT)]   
            diretional = (sensors[int(Sides.LEFT)] + sensors[int(Sides.RIGHT)] ) / 2

            error[int(Wheels.FL)] = - diretional  + angular + coef * 1.5
            error[int(Wheels.FR)] = + diretional  - angular + coef * 1.5
            error[int(Wheels.BL)] = + diretional  + angular + coef * 1.5
            error[int(Wheels.BR)] = - diretional  - angular + coef * 1.5
            
            motorControl.align(error)        
            r.sleep()
        
        # 2)
        rospy.loginfo("goToDock: 2) Go DOWN until lateral sensors get out of black line ")
        
        error = np.array([-2,+2,+2,-2])
        r = rospy.Rate(100)
        while(not rospy.is_shutdown()):

            sensors = linesensors.readLines()
            motorControl.align(error)            
    
            if( sensors[ int( Sides.LEFT) ] ==  2 and sensors[ int(Sides.RIGHT) ] ==  2 ):
                break
                        
            r.sleep()
  
        # 3) 
        rospy.loginfo("goToDock: 3) Go Down until find green line")

        linesensors.reset()
        is_stable = 0
        while(not rospy.is_shutdown()):
            sensors = linesensors.readLines()
            
            angular = sensors[int(Sides.LEFT)] - sensors[int(Sides.RIGHT)]   
            diretional = (sensors[int(Sides.LEFT)] + sensors[int(Sides.RIGHT)] ) / 2

            error[int(Wheels.FL)] = - diretional  + angular 
            error[int(Wheels.FR)] = + diretional  - angular 
            error[int(Wheels.BL)] = + diretional  + angular 
            error[int(Wheels.BR)] = - diretional  - angular 
            
            motorControl.align(error)        
        
            if( sensors[ int(Sides.LEFT)] == 0 and sensors[int(Sides.RIGHT)] == 0 ):
                is_stable += 1
                if( is_stable == 100):
                    break
            else:
                is_stable = 0
                        
            r.sleep()

        motorControl.stop()  

        rospy.loginfo("goToDock: 4) Success")
        
        return "succeeded"

class goToFirstPose(smach.State):

    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded', 'aborted'],
            input_keys=[],
            output_keys=['finalPose'])

        
    def execute(self, userdata):
        linesensors = LineSensor()
        motorControl = MotorControl(Kp=30, Kd=100, freq=100, momentum=0.6)
        
        # 1) Get through the green line
        rospy.loginfo("1) Get Through the Green Line")

        linesensors.reset(False)
        error = np.array([-2,+2,+2,-2])

        r = rospy.Rate(100)
        while(not rospy.is_shutdown()):

            sensors = linesensors.readLines()
            motorControl.align(error)            
    
            if( sensors[ int( Sides.LEFT) ] == - 2 and sensors[ int(Sides.RIGHT) ] == - 2 ):
                break
                        
            r.sleep()
  
        
        # 2) Align with the Next Line (Black)
        rospy.loginfo("2) Align with the Next Line Black")

        linesensors.reset(False)

        is_stable = 0
        while(not rospy.is_shutdown()):
            sensors = linesensors.readLines()
            
            angular = sensors[int(Sides.LEFT)] - sensors[int(Sides.RIGHT)]   
            diretional = (sensors[int(Sides.LEFT)] + sensors[int(Sides.RIGHT)] ) / 2

            error[int(Wheels.FL)] = - diretional  + angular 
            error[int(Wheels.FR)] = + diretional  - angular 
            error[int(Wheels.BL)] = + diretional  + angular 
            error[int(Wheels.BR)] = - diretional  - angular 
            
            motorControl.align(error)        
        
            if( sensors[ int(Sides.LEFT)] == 0 and sensors[int(Sides.RIGHT)] == 0 ):
                is_stable += 1
                if( is_stable == 100):
                    break
            else:
                is_stable = 0
                        
            r.sleep()

        motorControl.stop()        
        
        # 3 ) Go to the left following the line
        rospy.loginfo("3) Go to the Left Following the Line")

        linesensors.reset()

        while(not rospy.is_shutdown()):
            sensors = linesensors.readLines()
        
            angular = sensors[int(Sides.LEFT)] - sensors[int(Sides.RIGHT)]   
            diretional = (sensors[int(Sides.LEFT)] + sensors[int(Sides.RIGHT)] ) / 2

            error[int(Wheels.FL)] = - diretional  + angular - sensors[int(Sides.FRONT)] * 1
            error[int(Wheels.FR)] = + diretional  - angular - sensors[int(Sides.FRONT)] * 1
            error[int(Wheels.BL)] = + diretional  + angular - sensors[int(Sides.FRONT)] * 1
            error[int(Wheels.BR)] = - diretional  - angular - sensors[int(Sides.FRONT)] * 1
            
            if( sensors[int(Sides.FRONT)] == 0 and sensors[int(Sides.LEFT)] == 0 and sensors[int(Sides.RIGHT)]==0 ):
                is_stable = is_stable + 1
                
                if is_stable == 60:
                    break

            elif( sensors[int(Sides.FRONT)] != 0 and is_stable > 0):
                is_stable = 0

            motorControl.align(error)        

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
    """
        All strategy is in this class:

        1) check if all containers are known:

        IF THEY ARE NOT KNOWN: 
            A) If the container list is empty, this means that we just started the competition, and that we just perfomed the goToFirstPose.
            Now, we should recognize containers for the first time.

            B) If, some containers are already known, but not all the 32 containers, this mean that we need to go to the Blue intersection and
            recognize all containers there.
        
        
        2) Let's choose the best container to pick:
            The strategy is: 
                first, search for the container with the same color as the nearest dock. 
                second, search for the container with the other color without moving from intersection
                third, move from intersection and repeat one
                fourth, move from intersection and repeat second

        A better strategy would be to look for one color first.

        INPUTS:
            pose:
            containersList:
            containerColor:
            containerPose:

        OUTPUTS:
            pose:
            containerColor:
            containerPose:
            nextPose:



    """
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['containersUnknown', 'pick', 'changePosition','done', 'failed'],
                             input_keys=['pose', 'containersList', 'containerColor', 'containerPose'],
                             output_keys=['pose', 'containerColor','containerPose','nextPose'])

    def execute(self, userdata):
        
        # 1) Check if the containers are known
        known_containers = 0
        for l in userdata.containersList:
            known_containers = known_containers + len(l) 
        
        if known_containers == 0:
            # No containers are known, this mean that we just started our strategy and that we just perfomed the goToFirstPose.
            # Now, we recognize containers for the first time
            return 'containersUnknown' 
        elif known_containers < 30:
            if userdata.pose == Positions.BlueDock :
                return 'containersUnknown' 
            else:
                return  'changePosition'

        # rospy.Rate(1).sleep() ## Remove in real application, only for debbug 
        
        rospy.loginfo("Strategy Step: The pose of the robot is {}".format(userdata.pose))
           
        
        # 2) If they are known, choose which one is the best to pick
        if( userdata.pose == Positions.GreenIntersection ):
            firstColor = int(Colors.Green)
            secondColor = int(Colors.Blue)
            firstStackNumber = int(Positions.Container1)
        elif( userdata.pose == Positions.BlueIntersection ):
            firstColor = int(Colors.Blue)
            secondColor = int(Colors.Green)
            firstStackNumber = int(Positions.Container5)
        else:
            rospy.logerr("The strategyStep was called, but the robot position is {}".format(userdata.pose))
            return 'failed'

        for i in range(4):  # search for first Color first
            if userdata.containersList[i + firstStackNumber ][0] == firstColor :
                userdata.containerColor = firstColor
                userdata.containerPose = int(Positions( i + firstStackNumber))
                userdata.pose = userdata.pose
                rospy.loginfo("Strategy Step: Picked container {}".format(firstStackNumber))
                return 'pick'
        
        for i in range(4): # if no first Color container was found, look for the second Color
            if userdata.containersList[i+firstStackNumber][0] == secondColor :
                userdata.containerColor = secondColor
                userdata.containerPose = int(Positions( i + firstStackNumber))
                userdata.pose = userdata.pose
                return 'pick'

        rospy.loginfo("No container was found")
        # If no container was found in this intersection, check if there are containers in the next one to be picked
        firstStackNumber = 4 if firstStackNumber == 0 else 4

        for i in range(4):  
            if userdata.containersList[i + firstStackNumber ][0] == firstColor :
                userdata.containerColor = firstColor
                userdata.containerPose = int(Positions( i + firstStackNumber))
                userdata.pose = userdata.pose
                return 'changePosition'
        
        for i in range(4):
            if userdata.containersList[i+firstStackNumber][0] == secondColor :
                userdata.containerColor = secondColor
                userdata.containerPose = int(Positions( i + firstStackNumber))
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
        
        # install: pip install goprocam
        from goprocam import GoProCamera
        from goprocam import constants
        import time

        # Take the Picture
        gpCam = GoProCamera.GoPro()
        gpCam.downloadLastMedia(gpCam.take_photo(0), custom_filename="/tmp/filename.pic")

        # Process the Picutre

        userdata.nextPosition = 0  # given the actualPosition, olne can find the nextPosition
        pass

        if(True):
            return 'success'

class pickContainer(smach.State):
    """ This state picks and raises the container

        The servo pose should be already in the right side
    
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
        color = userdata.containersList[userdata.containerPose].pop(0)
        
        if not userdata.containersList[userdata.containerPose] :
            userdata.containersList[userdata.containerPose].append(Colors.Empty)

        if color == Colors.Green:
            userdata.containersList[int(Positions.GreenDock)].append(color) 
        elif color == Colors.Blue:
            userdata.containersList[int(Positions.BlueDock)].append(color) 
        else:
            rospy.logerr("The color to pick the container is wrong")
        
        # Code to pick the container:
        claw = Claw()
        claw.setGearAndPinionPose( len(userdata.containerList[userdata.containerPose]) + 1 , pickContainer=True)
        claw.controlElectromagnet(True)
        claw.resetGearAndPinionPose()

        return 'succeeded'

class dropContainer(smach.State):
    """ This state drops the container

        The servo pose should be already in the right side
    
        Inputs: 
            containersList: 
            containerPose:
        Output: 
            containersList: w/out the picked container

    """
    def __init__(self):
        smach.State.__init__(self, 
                            input_keys=['containersList','containerColor',],
                            outcomes=['preempted', 'succeeded','aborted'])
    
    def execute(self, userdata):
        
        # Code to drop the container:
        if userdata.containerColor == Colors.Green:
            height = len(userdata.containerList[ int(Positions.GreenDock )])
        elif userdata.containerColor == Colors.Blue:
            height = len(userdata.containerList[ int(Positions.BlueDock )])
        else:
            rospy.logerr("The color to drop the container is wrong")

        claw = Claw()
        claw.setGearAndPinionPose( height , pickContainer=True)
        claw.controlElectromagnet(turnOn=False)
        claw.resetGearAndPinionPose()

        return 'succeeded'

class whereToGo(smach.State):
    """
        This class navigate the robot after returning from the 
    """
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

class planner(smach.State):
    def __init__(self):
        pass
#print("RIGHT: {}  LEFT: {} ".format(sensors[int(Sides.RIGHT)], sensors[int(Sides.LEFT)]))
#print( "FL {}\tFR {}\nBL {}\tBR{}\n\n\n".format(                    - sensors[int(Sides.RIGHT)],                    - sensors[int(Sides.LEFT)],                    + sensors[int(Sides.LEFT)],                    + sensors[int(Sides.RIGHT)]) )

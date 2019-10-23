#!/usr/bin/env python
#coding: utf-8
import rospy
import smach_ros
import smach    

from std_msgs.msg import Float64, Bool
from robot_strategy.lineSensors import LineSensor, Sides
from robot_strategy.containerSensorsLib import ContainerSensors
from robot_strategy.motorControlLib import MotorControl, Wheels
from robot_strategy.utils import Colors, Positions, finalAlignmentIntersection
from robot_strategy.clawLib import Claw, GearAndPinionPoses

import numpy as np

from functools import partial

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

        userdata.robotPose = change_intersection(linesensors, motorControl, userdata.robotPose, 20)

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
      
        container_to_intersection(linesensors, motorControl)
            
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

        dock_to_intersection(linesensors, motorControl, userdata.containerColor, 50)
        
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
        
        to_container(linesensors, motorControl, containerSensors)
        
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
       
        to_dock(linesensors, motorControl, userdata.containerColor, 0)
        
        return "succeeded"

class goToFirstPose(smach.State):

    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded', 'aborted'],
            input_keys=[],
            output_keys=['finalPose'])

        
    def execute(self, userdata):
        linesensors = LineSensor()
        motorControl = MotorControl(freq=100)
        
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
            #rospy.loginfo("LEFT: {} RIGHT: {}".format( containerSensors.sensor[int(Sides.LEFT)], containerSensors.sensor[int(Sides.RIGHT)]))
            #rospy.loginfo("{}".format(rospy.get_time() - start))

            #print("RIGHT: {}  LEFT: {} FRONT: {}".format(sensors[int(Sides.RIGHT)], sensors[int(Sides.LEFT)], sensors[int(Sides.FRONT)]))
            #print( "FL {}\tFR {}\nBL {}\tBR {}\n\n\n".format(error[int(Wheels.FL)],                   
            #        error[int(Wheels.FR)],  error[int(Wheels.BL)], error[int(Wheels.BR)]) )



def strategyPlanner(container_list, inital_pose = Positions.GreenIntersection):
    """
    Primeiro:
        Checa se  o container é da cor que mais dá ponto. No caso em que os pontos são iguais, pega o mais próximo
    """
    position = inital_pose
    placesToGo=[]
    containersToPick=[]
    colorsToPick=[]
    for n in range(7) :

        green_value = sum( np.array(colorsToPick) == Colors.Green )
        blue_value  = sum( np.array(colorsToPick) == Colors.Blue)
        best_value = Colors.Green 
        
        if best_value == 6:
            break

        if green_value > blue_value:
            best_value = Colors.Green
        elif blue_value > green_value:
            best_value = Colors.Blue    
        else:
            if position  == Positions.GreenIntersection:
                best_value = Colors.Green
            else:
                best_value = Colors.Blue

        # Containers at the top:
        top_containers = np.array([ container[0]  for container in container_list[:4] ]) == best_value
        
        if sum(top_containers) == 0 : # check if any is of the  containers is the best value color
            if best_value == Colors.Green:
                best_value = Colors.Blue
            else:
                best_value = Colors.Green

            top_containers = np.array([ container[0]  for container in container_list[:4] ]) == best_value    
        index = -1
        if position == Positions.GreenIntersection:
            if top_containers[0]:
                index = 0
            elif top_containers[1]:
                index = 1
            elif top_containers[2]:
                index = 2
            elif top_containers[3]:
                index = 3
        else:
            if top_containers[2]:
                index = 2
            elif top_containers[3]:
                index = 3
            elif top_containers[0]:
                index = 0
            elif top_containers[1]:
                index = 1

        if index == -1 :
            return colorsToPick

        colorsToPick.append( container_list[index][0] )
        containersToPick.append(index)

        del container_list[index][0]
        
        if not container_list[index]:
            container_list[index].append(Colors.Empty)  

        position =  Positions.GreenIntersection if colorsToPick[-1] == Colors.Green else Positions.GreenIntersection

    return colorsToPick, containersToPick

def functionPlanner(colorsToPick, containersToPick, initialPose = Positions.GreenIntersection):

    position = initialPose

    functionList = []
    
    ln = LineSensor()
    mc = MotorControl()
    cc = Claw()
    cs = ContainerSensors()
    n_green = 0
    n_blue = 0
    for color, container_number in zip(colorsToPick, containersToPick):
        
        # Change position to stay in the right intersection
        if container_number in [0,1,5,6]:
            container_pose = Positions.GreenIntersection
        else:
            container_pose = Positions.BlueIntersection

        if container_pose != position: 
            functionList.append(partial( change_intersection, ln, mc, position) ) 
            position = Positions.BlueIntersection if position == Positions.GreenIntersection else Positions.GreenIntersection

        # Go to Container | # Pick container # Go Back to Intersection
        functionList.append( partial( pick_container, ln, cs, mc, cc, container_number ))

        # Change position to stay in the right intersection 
        if color == Colors.Green:
            container_pose = Positions.BlueIntersection
        else:
            container_pose = Positions.GreenIntersection
        
        if container_pose != position: 
            functionList.append(partial( change_intersection, ln, mc, position) ) 
            position = Positions.BlueIntersection if position == Positions.GreenIntersection else Positions.GreenIntersection

        # Go To Dock | Deposit Container | Go back to Intersection
        if color == Colors.Blue:
            n_blue += 1
            n_container = n_blue
        elif color == Colors.Green:
            n_green += 1
            n_container = n_green
        else:
            rospy.logerr("color of container is wrong")
            break
        
        if n_container == 1 :
            height = int( GearAndPinionPoses.container1 )
        elif n_container == 2:
            height = int( GearAndPinionPoses.container2 )
        elif n_container == 3:
            height = int( GearAndPinionPoses.container3 )
        elif n_container == 4:
            height = int( GearAndPinionPoses.container4 )
        elif n_container == 5:
            height = int( GearAndPinionPoses.container5 )
        else:
            rospy.logerr("There is no more height")
            break

        functionList.append( partial( drop_container, ln, cs, mc, cc, color,  ))

        return functionList
        
def change_intersection(linesensors, motorControl, robotStartingPose, STABLE_CONSTANT=0):
        
        # linesensors = LineSensor()
        # motorControl = MotorControl()
        motorControl.clear()
        error = np.array([0,0,0,0])
       
        if robotStartingPose == Positions.GreenIntersection:
            coef = -1     
            linesensors.reset() # we might not reset it... technically, the robot should be aligned before
        else:
             coef =  1 #check the side: should go to the right or left
             linesensors.reset(False)

        # 1) Get the front sensor out of the black line 
        rospy.loginfo("changeIntersection: 1)  Get out of black line | robotPose: {} {} {}".format(
            robotStartingPose, Positions.GreenIntersection, Positions.BlueIntersection
        ))
        
        motorControl.setParams(Kp=20, Kd=50, freq=100, momentum=0.6)
        r = rospy.Rate(100)
        r.sleep()
        sensors = linesensors.readLines()
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
        motorControl.clear()
        motorControl.setParams(Kp=30, Kd=50, freq=100, momentum=0.5)   

        if (coef == - 1 ):
            linesensors.reset(False)
        else:
            linesensors.reset()

        while(not rospy.is_shutdown()):
            sensors = linesensors.readLines()
        
            angular = sensors[int(Sides.LEFT)] - sensors[int(Sides.RIGHT)]   
            diretional = (sensors[int(Sides.LEFT)] + sensors[int(Sides.RIGHT)] ) / 2

            error[int(Wheels.FL)] = - diretional  + angular - sensors[int(Sides.FRONT)] * 2
            error[int(Wheels.FR)] = + diretional  - angular - sensors[int(Sides.FRONT)] * 2
            error[int(Wheels.BL)] = + diretional  + angular - sensors[int(Sides.FRONT)] * 2
            error[int(Wheels.BR)] = - diretional  - angular - sensors[int(Sides.FRONT)] * 2
            
            if( np.abs(sensors[int(Sides.FRONT)]) < 0.3 and np.abs(sensors[int(Sides.LEFT)]) < 0.3 and np.abs(sensors[int(Sides.RIGHT)]) < 0.3 ):
                is_stable = is_stable + 1
                motorControl.setParams(Kp=10, Kd=25, windUp=20., Ki=0., freq=100., momentum=0.6, deadSpace=15)
                if is_stable >= STABLE_CONSTANT:
                    break
            elif( sensors[int(Sides.FRONT)] != 0 and is_stable > 0):
                is_stable = 0

            motorControl.align(error)        
            r.sleep()

        motorControl.stop()
        
        robotFinalPose = int(Positions.BlueIntersection) if robotStartingPose == Positions.GreenIntersection else int(Positions.GreenIntersection)

        rospy.loginfo("changeIntersection: 3) Success. New pose is: {}".format(robotFinalPose))

        return robotFinalPose

def container_to_intersection(linesensors, motorControl):

        # linesensors = LineSensor()
        # motorControl = MotorControl()
        motorControl.clear()
        r = rospy.Rate(100)
        error = np.array([1.,1,1,1], dtype=float)

        # 2) Final Alignment
        rospy.loginfo("goFromContainerToIntersection: 1) Go Back until lateral sensors find blackline")
        motorControl.setParams(Kp=60., Kd=100., freq=100., momentum=0.6)        

        linesensors.reset()
        is_stable = 0
        while(not rospy.is_shutdown()):
            sensors = linesensors.readLines()
            
            angular = sensors[int(Sides.LEFT)] - sensors[int(Sides.RIGHT)]   
            diretional = (sensors[int(Sides.LEFT)] + sensors[int(Sides.RIGHT)] ) / 3.

            error[int(Wheels.FL)] =  + 1
            error[int(Wheels.FR)] =  - 1
            error[int(Wheels.BL)] =  - 1
            error[int(Wheels.BR)] =  + 1

            #rospy.loginfo(error)
            motorControl.align(error)        
        
            if( np.abs(sensors[ int(Sides.LEFT)]) < 0.3 or np.abs(sensors[int(Sides.RIGHT)]) < 0.3 ):
                is_stable += 1
                rospy.loginfo(is_stable)
                #if( is_stable == 1):
                break
            else:
                is_stable = 0
                        
            r.sleep()

        rospy.loginfo("goFromContainerToIntersection: 2) Success")
        motorControl.setParams(Kp=25., Kd=100., freq=100., momentum=0.6)        
        while(not rospy.is_shutdown()):
            sensors = linesensors.readLines()
        
            angular = ( sensors[int(Sides.LEFT)] - sensors[int(Sides.RIGHT)]   )
            diretional = (sensors[int(Sides.LEFT)] + sensors[int(Sides.RIGHT)] ) / 2

            error[int(Wheels.FL)] = - diretional  + angular - sensors[int(Sides.FRONT)] * 1.5
            error[int(Wheels.FR)] = + diretional  - angular - sensors[int(Sides.FRONT)] * 1.5
            error[int(Wheels.BL)] = + diretional  + angular - sensors[int(Sides.FRONT)] * 1.5
            error[int(Wheels.BR)] = - diretional  - angular - sensors[int(Sides.FRONT)] * 1.5

            if( np.abs(sensors[int(Sides.FRONT)]) < 0.3 and np.abs(sensors[int(Sides.LEFT)]) < 0.3 and np.abs(sensors[int(Sides.RIGHT)]) < 0.3 ):
                is_stable = is_stable + 1
                if is_stable >= 0:
                    break

            elif( sensors[int(Sides.FRONT)] != 0 and is_stable > 0):
                is_stable = 0

            motorControl.align(error)        

            r.sleep()
        motorControl.stop()  
        
        return 

def dock_to_intersection(linesensors, motorControl, containerColor, STABLE_CONSTANT=0):
        motorControl.clear()
        motorControl.setParams(Kp=30, Kd=100, freq=100, momentum=0.6, deadSpace=0)

        coef = -1 if containerColor == Colors.Green else 1 #check the side: should go to the right or left
        error = np.array([0,0,0,0])

        # 1) 
        rospy.loginfo("goFromDockToIntersection: 1) Go straight Up Until Lateral Sensors get out of line")        
        
        #linesensors.reset(False)
        error = np.array([-2,+2,+2,-2])
        error[int(Wheels.FL)] = - 2
        error[int(Wheels.FR)] = + 2
        error[int(Wheels.BL)] = + 2
        error[int(Wheels.BR)] = - 2

        r = rospy.Rate(100)
        r.sleep()

        while(not rospy.is_shutdown()):

            sensors = linesensors.readLines()
            motorControl.align(error)            
    
            if( np.abs(sensors[ int( Sides.LEFT) ]) == 1 and np.abs(sensors[ int(Sides.RIGHT) ]) ==  1 ):
                break
                        
            r.sleep()
  
        # 2) 
        rospy.loginfo("goFromDockToIntersection: 2) Go straight Up Until LEFT/RIGHT sensors find blackline")        
        linesensors.reset(False)
        motorControl.clear()
        is_stable = 0
        while(not rospy.is_shutdown()):
            sensors = linesensors.readLines()
            
            angular = (sensors[int(Sides.LEFT)] ) - ( sensors[int(Sides.RIGHT)] )  
            diretional = ((sensors[int(Sides.LEFT)]) + (sensors[int(Sides.RIGHT)]) ) / 2

            error[int(Wheels.FL)] = - diretional  + angular 
            error[int(Wheels.FR)] = + diretional  - angular 
            error[int(Wheels.BL)] = + diretional  + angular 
            error[int(Wheels.BR)] = - diretional  - angular 
            
            motorControl.align(error)        
        
            if( np.abs(sensors[ int(Sides.LEFT)]) < 0.3  and np.abs(sensors[int(Sides.RIGHT)])<0.3 ):
                is_stable += 1
                if( is_stable >= STABLE_CONSTANT):
                    break
            else:
                is_stable = 0
                        
            r.sleep()

        motorControl.stop() 
        
        # 3) 
        rospy.loginfo("goFromDockToIntersection: 3) Go to the side until find intersection, coef {}, Color {}( GREEN =={} )".format(coef, containerColor, int(Colors.Green)))
        motorControl.setParams(Kp=20, Kd=50, freq=100, momentum=0.5)
        linesensors._error[int(Sides.FRONT)] = 1 if containerColor == Colors.Green else - 1 #check the side: should go to the right or left

        while(not rospy.is_shutdown()): 
            sensors = linesensors.readLines()   

            angular = (sensors[int(Sides.LEFT)]) - (sensors[int(Sides.RIGHT)])  * 0.5
            diretional = ((sensors[int(Sides.LEFT)]) + (sensors[int(Sides.RIGHT)]) ) / 2

            error[int(Wheels.FL)] = - diretional  + angular  + coef * 2
            error[int(Wheels.FR)] = + diretional  - angular  + coef * 2
            error[int(Wheels.BL)] = + diretional  + angular  + coef * 2
            error[int(Wheels.BR)] = - diretional  - angular  + coef * 2

            if( np.abs(sensors[int(Sides.FRONT)]) < 0.3 ):
                is_stable = is_stable + 1
                #motorControl.setParams(Kp=10., Kd=0., windUp=0., Ki=0., freq=100, momentum=0.6)
                if is_stable >= STABLE_CONSTANT:
                    break

            elif( sensors[int(Sides.FRONT)] != 0 and is_stable > 0):
                is_stable = 0

            motorControl.align(error)        
            r.sleep()

        motorControl.stop()
        
        rospy.loginfo("goFromDockToIntersection: 4) Success")
        
        return 'succeeded'

def to_container(linesensors, motorControl, containerSensors):
    #TODO: the robot needs to get to the first row container and the second row of container
    linesensors.reset()
    error = np.array([1,1,1,1], dtype=float)

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

        if( sensors[ int( Sides.BACK) ] == 0 ):
            linesensors.reset()
            break
                    
        r.sleep()
    motorControl.stop()

    # 2) Keep going until find container
    rospy.loginfo("goToContainer: 2) Keep going until find container ( or 3 sec)")
    motorControl.setParams(Kp=20, Kd=100, freq=100, momentum=0.6, deadSpace=0)
    motorControl.clear()

    is_stable = 0
    while(not rospy.is_shutdown() ):
        sensors = linesensors.readLines()

        angular = sensors[int(Sides.FRONT)] - sensors[int(Sides.BACK)]   
        diretional = (sensors[int(Sides.FRONT)] + sensors[int(Sides.BACK)] ) / 2

        # angularC = 0 #int(containerSensors.sensor[1]) - int(containerSensors.sensor[0])
        #diretionalC = 2 - (int(containerSensors.sensor[0]) + int(containerSensors.sensor[1]))
        diretionalC = 1 - int(containerSensors.sensor[0])
        error[int(Wheels.FL)] =  + (- diretional  + angular )  - diretionalC # +angularC)
        error[int(Wheels.FR)] =  + (- diretional  - angular )  + diretionalC # -angularC)
        error[int(Wheels.BL)] =  + (- diretional  + angular )  + diretionalC # +angularC)
        error[int(Wheels.BR)] =  + (- diretional  - angular )  - diretionalC # -angularC)
        
        #rospy.loginfo(error)
        if( containerSensors.sensor[(0)] ):
            is_stable += 1

            if is_stable >= 1:
                break

        motorControl.align( error )
        r.sleep()

    rospy.loginfo("goToContainer: 3) Success")
    motorControl.stop()
    
    return 'succeeded'        

def to_dock(linesensors, motorControl, containerColor, STABLE_CONSTANT=0):
    
    motorControl.clear()
    motorControl.setParams(Kp=30, Kd=50, freq=100, momentum=0.5)
    coef = 1 if containerColor == Colors.Green else -1 #check the side: should go to the right or left

    error = np.array([0.,0.,0.,0.], dtype=float)

    # 1) 
    time = rospy.Time.now()
    rospy.loginfo("goToDock: 1) Go to the Side following the line during {}s | coef {}".format(1.5, coef))
    
    r = rospy.Rate(100)
    while(not rospy.is_shutdown() and 
        ( rospy.Time.now() - time < rospy.Duration(1.5) ) ):
        
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
    rospy.loginfo("goToDock: 1) Align before moving down".format(time))
    motorControl.setParams(Kp=30, Kd=50, Ki=10, freq=100, momentum=0.5)
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
    
        if( np.abs(sensors[ int(Sides.LEFT)]) < 0.3 and np.abs(sensors[int(Sides.RIGHT)]) < 0.3 ):
            is_stable += 1
            if( is_stable >= STABLE_CONSTANT):
                break
        else:
            is_stable = 0
                    
        r.sleep()

    motorControl.stop()         
    # 2)
    rospy.loginfo("goToDock: 2) Go DOWN until lateral sensors get out of black line ")
    motorControl.setParams(Kp=30, Kd=50, Ki=0, freq=100, momentum=0.5)
    error = np.array([2,-2,-2,+2], dtype=float)
    r = rospy.Rate(100)
    while(not rospy.is_shutdown()):

        sensors = linesensors.readLines()
        motorControl.align(error)            

        if( np.abs(sensors[ int( Sides.LEFT) ]) == 1 and np.abs(sensors[ int(Sides.RIGHT) ]) ==  1 ):
            break
                    
        r.sleep()

    # 3) 
    rospy.loginfo("goToDock: 3) Go Down until find green line")
    linesensors.reset()
    motorControl.setParams(Kp=20, Kd=0, Ki=0, freq=100, momentum=0.5, deadSpace=17)
    while(not rospy.is_shutdown()):
        sensors = linesensors.readLines()
        
        angular = (sensors[int(Sides.LEFT)]) - ( sensors[int(Sides.RIGHT) ])   
        diretional = ( ( sensors[int(Sides.LEFT)]) + (sensors[int(Sides.RIGHT)]) ) / 2.

        error[int(Wheels.FL)] = - diretional  + angular 
        error[int(Wheels.FR)] = + diretional  - angular 
        error[int(Wheels.BL)] = + diretional  + angular 
        error[int(Wheels.BR)] = - diretional  - angular 

        #rospy.loginfo(np.abs(sensors[ int(Sides.LEFT)] + K ))

    
    
        if( sensors[ int(Sides.LEFT)] !=  -1 and sensors[int(Sides.RIGHT)] != -1 ):
            motorControl.stop()
            break
        else:
            is_stable = 0

        motorControl.align(error )                        
        r.sleep()

    rospy.loginfo("goToDock: 3) Go Down until find green line")
    """
    linesensors.reset()
    rospy.Rate(1).sleep()
    is_stable = 0
    motorControl.clear()
    motorControl.setParams(Kp=20., Kd=50., windUp=20., Ki=0., freq=100., momentum=0.6)
    K = 0.7
    while(not rospy.is_shutdown()):
        sensors = linesensors.readLines()
        
        angular = (sensors[int(Sides.LEFT)] + 0.7) - ( sensors[int(Sides.RIGHT) ] + 0.7)   
        diretional = ( ( sensors[int(Sides.LEFT)] + 0.7) + (sensors[int(Sides.RIGHT)] + 0.7) ) / 2.

        error[int(Wheels.FL)] = - diretional  + angular 
        error[int(Wheels.FR)] = + diretional  - angular 
        error[int(Wheels.BL)] = + diretional  + angular 
        error[int(Wheels.BR)] = - diretional  - angular 

        rospy.loginfo(np.abs(sensors[ int(Sides.LEFT)] + K ))

        motorControl.align(error )        
    
        if( np.abs(sensors[ int(Sides.LEFT)] + K )  <= 0.3 or np.abs(sensors[int(Sides.RIGHT)] + K ) <= 0.3 ):
            is_stable += 1
            if( is_stable >= STABLE_CONSTANT + 10) :
                break
        else:
            is_stable = 0
    """                
    motorControl.stop()  

    rospy.loginfo("goToDock: 4) Success, coef {}".format(coef))
    
    return "succeeded"

def firstPose(linesensors,motorControl):
    # 1) Get through the green line
    rospy.loginfo("1) Get Through the Green Line")
    linesensors.reset(False)
    motorControl.setParams(Kp = 40)
    error = np.array([-3,+3,+3,-3])

    r = rospy.Rate(300)
    while(not rospy.is_shutdown()):

        sensors = linesensors.readLines()
        # motorControl.align(error)            
        rospy.loginfo("L: {} R: {}".format(sensors[Sides.LEFT], sensors[Sides.RIGHT])   )
        if( sensors[ int( Sides.LEFT) ] == - 1 and sensors[ int(Sides.RIGHT) ] == - 1 ):
            break
                    
        r.sleep()

    
    # 2) Align with the Next Line (Black)
    rospy.loginfo("2) Align with the Next Line Black")

    linesensors.reset(False)
    linesensors.error = error
    motorControl.clear()
    motorControl.setParams(Kp=30, Kd=100, momentum=0.7)
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
    
        if( np.abs(sensors[ int(Sides.LEFT)]) < 0.3 and np.abs(sensors[int(Sides.RIGHT)]) < 0.3 ):
            motorControl.setParams(Kp = 1, Kd= 0, Ki=0, windUp=0, momentum=0, deadSpace=15)
            is_stable += 1
            if( is_stable == 1):
                break
        else:
            is_stable = 0
                    
        r.sleep()

    motorControl.stop()        
    
    # 3 ) Go to the left following the line
    rospy.loginfo("3) Go to the Left Following the Line")
    linesensors.reset()
    motorControl.clear()
    motorControl.setParams(Kp=30, Kd=50, Ki=1, momentum=0.6)
    linesensors._error[int(Sides.FRONT)] = -2

    while(not rospy.is_shutdown()):
        sensors = linesensors.readLines()
    
        angular = ( sensors[int(Sides.LEFT)] - sensors[int(Sides.RIGHT)]   ) * 0.5
        diretional = (sensors[int(Sides.LEFT)] + sensors[int(Sides.RIGHT)] ) / 2

        error[int(Wheels.FL)] = - diretional  + angular - sensors[int(Sides.FRONT)] * 1.5
        error[int(Wheels.FR)] = + diretional  - angular - sensors[int(Sides.FRONT)] * 1.5
        error[int(Wheels.BL)] = + diretional  + angular - sensors[int(Sides.FRONT)] * 1.5
        error[int(Wheels.BR)] = - diretional  - angular - sensors[int(Sides.FRONT)] * 1.5
        
        if( np.abs(sensors[int(Sides.FRONT)]) < 0.3 and np.abs(sensors[int(Sides.LEFT)]) < 0.3 and np.abs(sensors[int(Sides.RIGHT)]) < 0.3 ):
            is_stable = is_stable + 1
            #motorControl.setParams(Kp=30, Kd=50, momentum=0.6)

            if is_stable >= 0:
                break

        elif( sensors[int(Sides.FRONT)] != 0 and is_stable > 0):
            is_stable = 0

        motorControl.align(error)        

        r.sleep()
    
    motorControl.stop()

    position = int(Positions.GreenIntersection)
    
    return position
    # Read container

    # Change Pose

    # Read container
    

def pick_container(linesensors, containersensors, motorControl, clawControl, container_number):
    # Go To Container
    to_container(linesensors, motorControl, containersensors)

    # Pick container
    angle_to_pick = 0 if container_number in [0,2] else 160

    clawControl.setServoPose(initial=90, end=angle_to_pick)
    clawControl.pickContainer()
    clawControl.resetGearAndPinionPose()
    clawControl.setServoPose(initial=angle_to_pick, end=90)

    # Go back to intersection
    container_to_intersection(linesensors, motorControl)

def drop_container(linesensors, containersensors, motorControl, clawControl, containerColor, height):
    # Go To Container
    to_dock(linesensors, motorControl, containerColor, STABLE_CONSTANT=10)

    # Drop container
    clawControl.dropContainer(height)
    clawControl.resetGearAndPinionPose()

    # Go back to intersection
    dock_to_intersection(linesensors, motorControl, containerColor, STABLE_CONSTANT=10)
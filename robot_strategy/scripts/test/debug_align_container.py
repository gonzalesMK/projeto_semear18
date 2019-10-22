#! /usr/bin/env python
import rospy
from robot_strategy.lineSensors import Sides, LineSensor
from robot_strategy.containerSensorsLib import ContainerSensors
from std_msgs.msg import Int64

if __name__ == '__main__':
    
    rospy.init_node('test2')

    linesensors = LineSensor()
    containersensors = ContainerSensors()
    linesensors.reset()
    r = rospy.Rate(100)
    pubClawPWM = rospy.Publisher("/front", Int64, queue_size=1)
    while(not rospy.is_shutdown()):
        
        sensors = linesensors.readLines()
        print("\t\tF: {}\nD: {}\tL: {}\t\t R: {}\tD: {}\n\t\tB: {}\n\n".format(
                sensors[int(Sides.FRONT)],
                containersensors.sensor[0],
                sensors[int(Sides.LEFT)],
                sensors[int(Sides.RIGHT)],
                containersensors.sensor[1],
                sensors[int(Sides.BACK)],
                ))
        
        pubClawPWM.publish(sensors[Sides.FRONT])
        r.sleep()

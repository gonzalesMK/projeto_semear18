#! /usr/bin/env python
import rospy
from robot_strategy.lineSensors import Sides, LineSensor


if __name__ == '__main__':
    
    rospy.init_node('test2')

    linesensors = LineSensor()
    linesensors.reset()
    r = rospy.Rate(10)

    while(not rospy.is_shutdown()):
        
        sensors = linesensors.readLines()
        print("\tF: {}\nL: {}\t\t R: {}\n\tB: {}\n\n".format(
                sensors[int(Sides.FRONT)],
                sensors[int(Sides.LEFT)],
                sensors[int(Sides.RIGHT)],
                sensors[int(Sides.BACK)],
                ))
        r.sleep()

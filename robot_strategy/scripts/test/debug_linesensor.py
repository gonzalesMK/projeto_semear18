#! /usr/bin/env python
import rospy
from robot_strategy.lineSensors import Sides, LineSensor, sensorSides
from robot_strategy.containerSensorsLib import ContainerSensors
from std_msgs.msg import Int64

if __name__ == '__main__':
    
    rospy.init_node('test2')

    linesensors = LineSensor()
    containersensors = ContainerSensors()
    linesensors.reset()
    r = rospy.Rate(100)
    pubClawPWM = rospy.Publisher("/front", Int64, queue_size=1)

    pFL = rospy.Publisher('/readingsFL', Int64, queue_size=10)  
    pFR = rospy.Publisher('/readingsFR', Int64, queue_size=10)  
    pBL = rospy.Publisher('/readingsBL', Int64, queue_size=10)  
    pBR = rospy.Publisher('/readingsBR', Int64, queue_size=10)  
    pLF = rospy.Publisher('/readingsLF', Int64, queue_size=10)  
    pLB = rospy.Publisher('/readingsLB', Int64, queue_size=10)  
    pRF = rospy.Publisher('/readingsRF', Int64, queue_size=10)  
    pRB = rospy.Publisher('/readingsRB', Int64, queue_size=10)  

    pF = rospy.Publisher('/readingsF', Int64, queue_size=10)  
    pL = rospy.Publisher('/readingsL', Int64, queue_size=10)  
    pR = rospy.Publisher('/readingsR', Int64, queue_size=10)  
    pB = rospy.Publisher('/readingsB', Int64, queue_size=10)  

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
        
        #print(linesensors._readings)

        pFL.publish( linesensors._readings[sensorSides.FL])
        pFR.publish( linesensors._readings[sensorSides.FR])
        pBL.publish( linesensors._readings[sensorSides.BL])
        pBR.publish( linesensors._readings[sensorSides.BR])
        pLF.publish( linesensors._readings[sensorSides.LF])
        pLB.publish( linesensors._readings[sensorSides.LB])
        pRF.publish( linesensors._readings[sensorSides.RF])
        pRB.publish( linesensors._readings[sensorSides.RB])

        pR.publish( sensors[int(Sides.RIGHT)])
        pL.publish( sensors[int(Sides.LEFT)])
        pB.publish( sensors[int(Sides.BACK)])
        pF.publish( sensors[int(Sides.FRONT)])
        
        r.sleep()

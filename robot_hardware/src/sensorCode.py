#!/usr/bin/env python
import rospy
import serial
import numpy as np
from std_msgs.msg import Int64,UInt8

if __name__ == "__main__":
    
    rospy.init_node('pyserialTest')

    pFL = rospy.Publisher('/python/pololuSensorFL', UInt8, queue_size=10)  
    pFR = rospy.Publisher('/python/pololuSensorFR', UInt8, queue_size=10)  
    pBL = rospy.Publisher('/python/pololuSensorBL', UInt8, queue_size=10)  
    pBR = rospy.Publisher('/python/pololuSensorBR', UInt8, queue_size=10)  
    pLF = rospy.Publisher('/python/pololuSensorLF', UInt8, queue_size=10)  
    pLB = rospy.Publisher('/python/pololuSensorLB', UInt8, queue_size=10)  
    pRF = rospy.Publisher('/python/pololuSensorRF', UInt8, queue_size=10)  
    pRB = rospy.Publisher('/python/pololuSensorRB', UInt8, queue_size=10)  
    
    pubLimitSwitchs = rospy.Publisher("/claw/limitSwitchs",UInt8 , queue_size=1)
    pubEncoder = rospy.Publisher("/claw/height",Int64 , queue_size=1)
    pubContainers = rospy.Publisher("/containerSensor",UInt8 , queue_size=1)

    while( not rospy.is_shutdown() ):
        try :
            with serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1) as ser:
                while( not rospy.is_shutdown() ):
                    b = np.array( ser.read(13), dtype= np.int64)

                    pFL.publish(b[0])
                    pFR.publish(b[1])
                    pBL.publish(b[2])
                    pBR.publish(b[3])
                    pLF.publish(b[4])
                    pLB.publish(b[5])
                    pRF.publish(b[6])
                    pRB.publish(b[7])

                    pubContainers.publish(b[8])
                    pubLimitSwitchs.publish(b[9])
                    
                    pubEncoder.publish( -( b[10] + b[11] * 256 + b[12] * 65536 + b[13] * 16777216 - 2147483648) )
        except:
            print("Trying again soon")
            rospy.Rate(1).sleep()    

#!/usr/bin/env python
import rospy
import serial
import struct
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
    b = [0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    while( not rospy.is_shutdown() ):
            with serial.Serial('/dev/ttyUSB0', 115200, timeout=.01) as ser:
                while( not rospy.is_shutdown() ):
                    a = struct.pack('!B', 68)
                    ser.write(a)
                    a = str(ser.read(14))
                    
                    
                    if len(a) == 14:
                        j = 0
                        for i in a:
                            try:
                                b[j] = ord( i)
                                #int_values = struct.unpack('>13B', a)        
                                j = j + 1
                            except :
                                print("exception")
                                pass
                        
                        print("j: {}".format(j))
                        
                        if j :
                            b = np.array( b, dtype= np.int64)
                            print(b)
                            print( "A0: {} A1: {} A2: {} A3: {} A4: {} A5: {} A6: {} A7: {}".format( b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]))
                            print("Containers: {}".format(b[8]))
                            print("Limit: {}".format([9]))
                            print("encoder: {}".format( -( b[10] + b[11] * 256 + b[12] * 65536 + b[13] * 16777216 - 2147483648) ))

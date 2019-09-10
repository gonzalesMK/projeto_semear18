#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
Esse código deve ser responsável por checar se o número de rotações do Encoder lida pela Rasp é condizente com o Datasheet:

Link para página do motor:  https://www.pololu.com/product/4868
Contagens por revolução: 8245.92 counts per revolution

Procedimento:
    Esse código irá controlar a roda para ela dar 10 voltas, ou, 82459.2 contagens de encoder. 
    Uma pessoa deve ficar olhando a roda e certificar-se de que ela girou 10 voltas e quão próximo da posição esperada ela está.

* é necesssário rodar o código da RASP: publishEncoder e a interface com o arduino motorCode
"""

import rospy
from std_msgs.msg import Float64, Bool
from robot_strategy.motorControlLib import Wheels
import numpy as np
encoderReadings = np.array( [ [0], [0], [0], [0]] ) 
totalRev = 82459
K = 0.08

def callback(data, args):
    args[0][0] = data.data

if __name__ == "__main__":

    rospy.init_node('testar_encoder')
    #  Subscribers: 

    #  Encoder channels
    rospy.Subscriber("/motorFR/encoderVelocity",Float64,callback, (encoderReadings[Wheels.FR], ))
    rospy.Subscriber("/motorFL/encoderVelocity",Float64,callback, (encoderReadings[Wheels.FL], ))
    rospy.Subscriber("/motorBR/encoderVelocity",Float64,callback, (encoderReadings[Wheels.BR], ))
    rospy.Subscriber("/motorBL/encoderVelocity",Float64,callback, (encoderReadings[Wheels.BL], ))

    #  Publishers:
    
    #  Encoder's Enable 
    pub_encoderEnable = rospy.Publisher('/encoder_enable', Bool, queue_size=10)  
    
    #  PWM controller
    pubList = [0,0,0,0]
    pubList[Wheels.FL] = rospy.Publisher('/motorFL/pwm', Float64, queue_size=10)    
    pubList[Wheels.BL] = rospy.Publisher('/motorBL/pwm', Float64, queue_size=10)    
    pubList[Wheels.FR] = rospy.Publisher('/motorFR/pwm', Float64, queue_size=10)    
    pubList[Wheels.BR] = rospy.Publisher('/motorBR/pwm', Float64, queue_size=10)


    # Turn On Encoder
    pub_encoderEnable.publish(True)
    rospy.sleep(1)
    pub_encoderEnable.publish(True)

    rospy.sleep(0.50)
    initEncoder = encoderReadings.flatten() # Initial encoder Reading
    rospy.loginfo("Init: {}".format(initEncoder))
    
    goal = initEncoder + totalRev # Goal of Encoder Readings
    rospy.loginfo("Goal: {}".format(goal))
    
    # Control Loop
    error = np.array([0,0,0,0])
    old_now = rospy.Time.now()
    now = rospy.Time.now()
    while not rospy.is_shutdown():

        # Calculate Error
        error = goal - encoderReadings.flatten()

        # Calculate Response
        actuation = error * K
        
        actuation = np.clip(actuation, -150, 150)
        
        # Send i 
        for act, pub in zip(actuation, pubList):
            pub.publish(act)

        if now - old_now > rospy.Duration(0.5):
            rospy.loginfo("Error is {} and Actuation is {}".format(error, actuation))
            old_now = now

        now = rospy.Time.now()
        rospy.sleep(0.01)
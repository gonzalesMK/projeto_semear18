#! /usr/bin/env python3
import rospy
import actionlib

import numpy as np
from std_msgs.msg import Bool

from goprocam import GoProCamera
from goprocam import constants

import time

from functools import partial


def cb(gpCam, msg):

    gpCam.downloadLastMedia(gpCam.take_photo(0), custom_filename="/tmp/filename.jpg")


if __name__ == '__main__':
    rospy.init_node('testeGoPro')

    # Take the Picture
    gpCam = GoProCamera.GoPro()
    rospy.loginfo("Go Pro connected")
    rospy.Subscriber( "/takePicture", Bool, partial( cb, gpCam ))

    rospy.spin()
    


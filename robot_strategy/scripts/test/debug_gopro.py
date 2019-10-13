#! /usr/bin/env python3
import rospy
import actionlib

import numpy as np

from goprocam import GoProCamera
from goprocam import constants
import time


if __name__ == '__main__':
    rospy.init_node('testeGoPro')


    # Take the Picture
    rospy.loginfo("hey")
    gpCam = GoProCamera.GoPro()
    gpCam.downloadLastMedia(gpCam.take_photo(0), custom_filename="/tmp/filename.jpg")


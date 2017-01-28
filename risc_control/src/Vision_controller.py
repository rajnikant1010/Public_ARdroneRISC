#!/usr/bin/env python

'''======================================================
    Created by:      Li Yin
    Last updated:    March 2015
    File name:       Vision_controller.py
    Organization:    RISC Lab, Utah State University
    Notes:	     ECE6930 Final Project

    ======================================================'''

    #================================#
    #    Libraries/modules Needed    #
    #================================#

import roslib; roslib.load_manifest('ardrone_tutorials')
roslib.load_manifest('risc_msgs')
import rospy
from  math import *
import rospkg
import numpy as np
import scipy.linalg as la
import time

    #=======================#
    #    Messages Needed    #
    #=======================#

from risc_msgs.msg import *

    #========================#
    #        Globals         #
    #========================#
yaw_kp = 1.5
throttle_kp = .5
LM = Landmark()
LM_old = 0
width = 640
height = 360
    #==================#
    #    Publishers    #
    #==================#

pub_ctrl = rospy.Publisher('/controls', Controls, queue_size = 1)

    #================#
    #    Get Rois    #
    #================#

def GetRois(S):

    global LM
    for i in range(len(S.Obj[0].landmarks)):
       if S.Obj[0].landmarks[i].name == 'pink':
           LM = S.Obj[0].landmarks[i]
 
    #========================#
    #    Basic Controller    #
    #========================#

def Basic_Controller():
    global LM, yaw_kp, width, height, throttle_kp
    g = 9.81
    m = .45
    Ctrl = Controls()
    # set controls object array length to 1
    Ctrl.Obj = [Control()]*1

        #===================#
        #    Get  Errors    #
        #===================#
    if LM.x != LM_old:
        yaw_error = -(LM.x - .5*width)/(.5*width)
    else:
        yaw_error = 0

    if LM.y != LM_old:
        T_error = -(LM.y - .5*height)/(.5*height)
    else:
        T_error = 0

        #==================#
        #   Set Controls   #
        #==================#
    psi_cmd = yaw_kp*yaw_error
    T_cmd = T_error*throttle_kp
        # Controls for Ardrone
        # -phi = right... +phi = left
        # -theta = back... +theta = forward
        # -psi = right...  +psi = left
    ctrl        = Control()
    ctrl.name   = "quad"
    ctrl.phi    = 0
    ctrl.theta  = 0
    ctrl.psi    = psi_cmd
    ctrl.T      = g*m+T_cmd
    Ctrl.Obj[0] = ctrl
    pub_ctrl.publish(Ctrl)

    #===================#
    #       Main        #
    #===================#

if __name__=='__main__':
    import sys
    rospy.init_node('Image_controller')

    #=====================================#
    #    Set up Publish/Subscribe Loop    #
    #=====================================#

    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        sub_rois  = rospy.Subscriber('/land_rois' , Observed_rois, GetRois)
        Basic_Controller()
        r.sleep()

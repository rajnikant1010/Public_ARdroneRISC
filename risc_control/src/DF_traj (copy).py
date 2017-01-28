#!/usr/bin/env python

'''======================================================
                    ICUAS 2015 Trajectory
   ======================================================'''

import roslib; 
roslib.load_manifest('risc_msgs')
import rospy
from math import *
import time
import numpy as np
    #=======================#
    #    Messages Needed    #
    #=======================#

from risc_msgs.msg import *
from std_msgs.msg import *


    #========================#
    #        Globals         #
    #========================#

# enable time, pi and publisher
start_time = 0
PI         = 3.141592653589793
pub        = rospy.Publisher('trajectory',Trajectories,queue_size = 1)


# Select Trajectory Variables
period     = 8 #  8 seconds for Circle, 9 for fig-8
a          = 0.8 # 0.8 best for circle, 0.8 best for fig-8
b          = 0.8 # 0.8 best for circle, 0.6 best for fig-8
c          = 0
n          = 1
w1         = 2*PI/period
w2         = w1
w3         = w1



    #====================================#
    #    Update and Publish Trajectory   #
    #====================================#

def Datahandler():
    global start_time, PI, pub, period, a, b, c, n, w1, w2, w3
    WP 		= Trajectories()
    WP.Obj 	= [Trajectory()]*1
    time_now 	= rospy.get_time()
    t 		= time_now-start_time
    #   
    #  
    #    #=================#
    #    #    Trajectory   #
    #    #=================#
    #
    traj = Trajectory()
         # Position
    traj.x       = a*cos(w2*t)
    traj.y       = b*sin(w1*t)
    traj.z       = n+c*sin(w3*t)
    traj.psi     = 0
         # Velocity
    traj.xdot    = -a*w2*sin(w2*t)
    traj.ydot    = b*w1*cos(w1*t)
    traj.zdot    = c*w3*cos(w3*t)
    traj.psidot  = 0
         # Acceleration
    traj.xddot   = -a*w2*w2*cos(w2*t)
    traj.yddot   = -b*w1*w1*sin(w1*t)
    traj.zddot   = -c*w3*w3*sin(w3*t)
    traj.psiddot = 0
        
    #==================#
    #     Publish      #
    #==================#

    WP.Obj = [traj]
    pub.publish(WP)
    


    #===================#
    #       Main        #
    #===================#

if __name__=='__main__':
    rospy.init_node('virtual_target')
    start_time = rospy.get_time()

    r = rospy.Rate(200)
    while not rospy.is_shutdown():
        Datahandler()
        r.sleep()
    rospy.loginfo("Virtual Target Node Has Shutdown.")
    rospy.signal_shutdown(0)

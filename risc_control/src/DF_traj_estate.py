#!/usr/bin/env python

'''======================================================
                    ICUAS 2015 Trajectory
   ======================================================'''

import roslib; roslib.load_manifest('ardrone_tutorials')
roslib.load_manifest('risc_msgs')
import rospy
from math import *
import cv2
import time
import numpy as np
    #=======================#
    #    Messages Needed    #
    #=======================#

from risc_msgs.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import Joy

    #========================#
    #        Globals         #
    #========================#

# enable time, pi and publisher
start_time = 0
mode       = 0
PI         = 3.141592653589793
pub        = rospy.Publisher('trajectory',Trajectories,queue_size = 1)
#pub_vel    = rospy.Publisher('target_velocity',Float64,queue_size = 200)


# Select Trajectory Variables
period     = 8 #seconds for circle
a          = 0.8
b          = a
c          = 0
n          = 1
w1         = 2*PI/period
w2         = w1
w3         = w1
#vt         = 0.2 # m/s
#V_veh = 0.3
#Veh_init_X = 0.5643
#Veh_init_Y = 1.2213
#phase      = PI/2 


    #======================================#
    #    Parameters never to be changed    #
    #======================================#

#Des_X      = 0.5643    # Please Never Change
#Des_Y      = 1.2213   # Please Never Change


# initial position of the quad
#STATES = States()
#start_state = States()
#init = True


    #===============#
    #    Get Joy    #
    #===============#

def GetJoy(joy):
    global start_time, mode
    if mode == 0:
        start_time = rospy.get_time()
    mode    = mode + joy.buttons[1]
    #print mode

    #=========================#
    #    Get Initial State    #
    #=========================#

#def GetStates(X):
 #   global STATES,start_state, init, mode
 #   STATES = X.Obj[0]
 #   if init and mode != 0:
 #       start_state = X.Obj[0]
 #       init = False
    #====================================#
    #    Update and Publish Trajectory   #
    #====================================#

def Datahandler():
    global start_time, PI, pub, period, a, b, c, n, w1, w2, w3, mode
    WP 		= Trajectories()
    WP.Obj 	= [Trajectory()]*1
    time_now 	= rospy.get_time()
    t 		= time_now-start_time
    #Xv_init     = start_state.x
    #Yv_init     = start_state.y
    
    
    if mode == 0:

        #=================#
        #    Trajectory   #
        #=================#

        traj = Trajectory()
        traj.name = "WP"
        # Position
        traj.x       = 0
        traj.y       = 0.7
        traj.z       = n
        traj.psi     = 0        
        # Velocity
        traj.xdot    = 0
        traj.ydot    = 0
        traj.zdot    = 0
        traj.psidot  = 0
        # Acceleration
        traj.xddot   = 0
        traj.yddot   = 0
        traj.zddot   = 0
        traj.psiddot = 0
        
    else:
        #=================#
        #    Trajectory   #
        #=================#

        traj = Trajectory()
        traj.name = "DF"
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
    #pub_vel.publish(vt)


    #===================#
    #       Main        #
    #===================#

if __name__=='__main__':
    # Firstly we setup a ros node, so that we can communicate with the other packages
    rospy.init_node('virtual_target')
    start_time = rospy.get_time()
    #=====================================#
    #    Set up Publish/Subscribe Loop    #
    #=====================================#
    #sub_states = rospy.Subscriber('/cortex_raw', Cortex, GetStates)
    r = rospy.Rate(200)
    while not rospy.is_shutdown():
        #sub_states = rospy.Subscriber('/cortex_raw', Cortex, GetStates)
        sub = rospy.Subscriber('/joy', Joy, GetJoy)
        Datahandler()
        r.sleep()
    # and only progresses to here once the application has been shutdown
    rospy.loginfo("Virtual Target Node Has Shutdown.")
    rospy.signal_shutdown(0)

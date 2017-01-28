#!/usr/bin/env python

'''======================================================
        Trajectory Shaping AIAA SCITECH (26 MAY 2015)
   ======================================================'''

import roslib; roslib.load_manifest('ardrone_tutorials')
roslib.load_manifest('risc_msgs')
import rospy
from math import *
import cv2
import time
import numpy as np
import scipy.linalg as la

    #=======================#
    #    Messages Needed    #
    #=======================#

from risc_msgs.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
    #========================#
    #        Globals         #
    #========================#

# enable time, pi and publisher
start_time = 0
mode       = 0
PI         = 3.141592653589793
pub        = rospy.Publisher('trajectory',Trajectories,queue_size = 1)
#pub_Range       = rospy.Publisher('/range', Float64, queue_size = 200)
pub_vt   = rospy.Publisher('target_velocity',Float64,queue_size = 200)


# Select Trajectory Variables
a          = 0.8
b          = a
c          = 0
n          = 1
V_veh      = 0.1
#vt         = V_veh
R_star     = 0.4
Veh_init_X = R_star+0.15
Veh_init_Y = a
phase      = PI/2


    #======================================#
    #    Parameters never to be changed    #
    #======================================#

Des_X      = 0.5643   
Des_Y      = 1.2213   


# initial position of the quad
STATES = States()
start_state = States()
init = True


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

def GetStates(X):
    global STATES,start_state, init, mode
    STATES = X.Obj[0]
    if init and mode != 0:
        start_state = X.Obj[0]
        init = False


    #=========================#
    #    Get Range  #
    #=========================#

def GetRange(X):
    global Range
    Range = X
   

    #====================================#
    #    Update and Publish Trajectory   #
    #====================================#

def Datahandler():
    global start_time, PI, pub, a, b, c, n, w1, w2, w3, mode,Veh_init_X,Veh_init_Y,X_offset,Y_offset,phase,STATES,start_state,Des_X,Des_Y,R_star,V_veh,Range,pub_vt#,pub_Range#,vt
    WP 		= Trajectories()
    WP.Obj 	= [Trajectory()]*1
    time_now 	= rospy.get_time()
    t 		= time_now-start_time
    Xv_init     = start_state.x
    Yv_init     = start_state.y

    #Veh_pos = np.asmatrix(np.zeros((3,1)))
    #Veh_pos[0] = STATES.x
    #Veh_pos[1] = STATES.y
    #Veh_pos[2] = STATES.z
    xx = STATES.x
    yy = STATES.y
    zz = STATES.z  
    Veh_V = np.asmatrix(np.zeros((3,1)))
    Veh_V[0] = STATES.u
    Veh_V[1] = STATES.v
    Veh_V[2] = STATES.w
    V_v = np.linalg.norm(Veh_V) 
    #print V_v 
    
    if t < 0.2: # This time should be multiples of 0.05 and cannot be less than 0.05, otherwise this node will shut down.
        RANGE = R_star # Formula for Initial Range has to be defined here.
    else:
        RANGE = Range.data
    #print RANGE
    
    if mode == 0:
       
        #=================#
        #    Trajectory   #
        #=================#

        traj = Trajectory()
        traj.name = "WP"
        # Position        
        traj.x       = Veh_init_X 
        traj.y       = Veh_init_Y 
        traj.z       = n
        traj.psi     = 0
        #-3.14/2
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
        vt = V_veh
    else: 
        
        #Correction_X = (Xv_init-Des_X)
        #Correction_Y = (Yv_init-Des_Y)
        
        if RANGE <0.008*R_star:
            LOS = R_star
        else:
            LOS = RANGE

        
        #print LOS
        vt = (V_veh*R_star)/LOS # CHECK IF (V_v) is Working or (V_veh) would work
        #if vt > 1.5*V_veh:
        #    Tar_vel = 0
        #else:
        #    Tar_vel = vt    
        #print vt
        

        vt = np.convolve(vt, 0.5)
        w  = vt/a
        


        #=================#
        #    Trajectory   #
        #=================#

        traj = Trajectory()
        traj.name = "SWp"
        # Position
                
        traj.x       = a*cos(phase+w*t)       #Correction_X+
        traj.y       = b*sin(phase+w*t)      #Correction_Y+
        traj.z       = n+c*sin(w*t)
        traj.psi     = 0
        # Velocity
        traj.xdot    = -a*w*sin(w*t)
        traj.ydot    = b*w*cos(w*t)
        traj.zdot    = c*w*cos(w*t)
        traj.psidot  = 0
        # Acceleration
        traj.xddot   = -a*w*w*cos(w*t)
        traj.yddot   = -b*w*w*sin(w*t)
        traj.zddot   = -c*w*w*sin(w*t)
        traj.psiddot = 0
        #print yy
        #Rxy   = sqrt((traj.x-xx)*(traj.x-xx)+(traj.y-yy)*(traj.y-yy))
    #==================#
    #     Publish      #
    #==================#

    WP.Obj = [traj]
    pub.publish(WP)
    pub_vt.publish(vt)
    

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
        sub_states = rospy.Subscriber('/cortex_raw', Cortex, GetStates)
        sub = rospy.Subscriber('/joy', Joy, GetJoy)        
        blub_rng = rospy.Subscriber('/range', Float64, GetRange)
        Datahandler()               
        r.sleep()
    # and only progresses to here once the application has been shutdown
    rospy.loginfo("Virtual Target Node Has Shutdown.")
    rospy.signal_shutdown(0)

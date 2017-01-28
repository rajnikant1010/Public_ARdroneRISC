#!/usr/bin/env python

'''======================================================
 NEW DF PARW_ABHI
 ======================================================'''

import roslib;
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
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Empty
from std_msgs.msg import Float64

    #========================#
    #        Globals         #
    #========================#

PI 	        = 3.141592653589793
Threshold 	= 1000
states          = Cortex()
states.Obj      = [States()]*1
euler_max       = 0.349066 #in radians
max_yaw_rate    = 0.3490659 #in radians/sec
max_alt_rate    = 1000     # in mm/sec
rate            = 200      # Hz
start_time      = 0

    #==================#
    #    Publishers    #
    #==================#

pubTakeoff      = rospy.Publisher('/ardrone/takeoff',Empty, queue_size = 1)
pub_ctrl        = rospy.Publisher('/controls', Controls, queue_size = 1)

    #=====================#
    #    Gain Matrices    #
    #=====================#



K = np.matrix([[0.45, 0, 0, 0.45, 0, 0, 0],[0, 0.45, 0, 0, 0.45, 0, 0],[ 0, 0, -0.6, 0, 0, -6.6, 0],[0, 0, 0, 0, 0, 0, 1]]) #circle
#K = np.matrix([[0.27, 0, 0, 0.8, 0, 0, 0],[0, 0.27, 0, 0, 0.8, 0, 0],[0, 0, -0.6, 0, 0, -6.6, 0],[0, 0, 0, 0, 0, 0, 1]])#flat_8
#K = np.matrix([[0.5, 0, 0, 0.5, 0, 0, 0],[0, 0.5, 0, 0, 0.5, 0, 0],[ 0, 0, -0.6, 0, 0, -6.6, 0],[0, 0, 0, 0, 0, 0, 1]]) 

    #===================================#
    #    Radians between + or - pi/2    #
    #===================================#

def pi2pi(angle):
     if abs(angle)>PI/2:
           if angle>0:
                angle = angle-PI
           else:
                angle = angle+PI
     return angle

    

    #=====================#
    #    Get Trajectory   #
    #=====================#
def GetTrajectory(W):

    traj = W
    Basic_Controller(traj,K)


    #========================#
    #    Get Cortex States   #
    #========================#

def GetStates(S):

    global states
    states = S

   
    #========================#
    #    Basic Controller    #
    #========================#

def Basic_Controller(traj,K):
    global states,PI, euler_max, max_yaw_rate, max_alt_rate, pub_ctrl
    #rospy.loginfo("In Basic controller")
    Ctrl        = Controls()
    # Initiate Control Messages  
    Ctrl.Obj = [Control()]*1
    Ctrl.header.stamp = states.header.stamp
    g = 9.81
    m = 0.736 # 0.736Kg MiniQuad mass along with Battery 

        #===================================#
        #    Get State Trajectory Errors    #
        #===================================#
    #
    if states.Obj[0].visible:
        X = np.asmatrix(np.zeros((7,1)))
        X[0] = traj.Obj[0].x-states.Obj[0].x
        X[1] = traj.Obj[0].y-states.Obj[0].y
        X[2] = traj.Obj[0].z-states.Obj[0].z
        X[3] = traj.Obj[0].xdot-states.Obj[0].u
        X[4] = traj.Obj[0].ydot-states.Obj[0].v
        X[5] = traj.Obj[0].zdot-states.Obj[0].w
        X[6] = pi2pi(traj.Obj[0].psi)-states.Obj[0].psi*PI/180

        #============================================#
        #     Differential Flatness Control Input    #
        #============================================#

        # LQR input
        utilde = -K*X
        # required input
        #    Messages Needed    #
        u_r = np.matrix([[traj.Obj[0].xddot],[traj.Obj[0].yddot],[traj.Obj[0].zddot],[traj.Obj[0].psiddot]])
        u = utilde-u_r+np.matrix([[0],[0],[9.81],[0]])

        #==================================#
        #     Rotate to Vehicle 1 Frame    #
        #==================================#

        psi = states.Obj[0].psi*PI/180 #psi is negative in first quadrant
        rotZ = np.matrix([[cos(-psi), -sin(-psi), 0],[sin(-psi), cos(-psi), 0],[0, 0, 1]])
        Cart = np.matrix([[-1, 0, 0],[0, -1, 0],[0, 0, 1]]) # fix x and y directions
        u[:-1] = Cart*rotZ*u[:-1]

        #u = np.matrix([[0],[-0.1],[9.8],[0]])
        #print u

        #===================================#
        #     Normalize given the Thrust    #
        #===================================#

        T = sqrt(u[0,-1]*u[0,-1]+u[1,-1]*u[1,-1]+u[2,-1]*u[2,-1])
        u[:-1] = np.divide(u[:-1],T)
        #==================#
        #   Set Controls   #
        #==================#

        # Controls for Ardrone
        # -phi = right... +phi = left
        # -theta = back... +theta = forward
        # -psi = right...  +psi = left
        ctrl        = Control()
        ctrl.name   = states.Obj[0].name
        #ctrl.phi    = atan2(u[1,-1],u[2,-1])
        ctrl.phi    = asin(u[1,-1])
        #ctrl.theta  = asin(u[0,-1])
        ctrl.theta  = atan2(u[0,-1],u[2,-1])
        ctrl.psi    = u[3,-1]
        ctrl.T      = T*m
        Ctrl.Obj[0] = ctrl
        Ctrl.header = states.header
        pub_ctrl.publish(Ctrl)



    #===================#
    #       Main        #
    #===================#

if __name__=='__main__':
    import sys
    rospy.init_node('LQR')#LQR Name doesnt affect anything.....
   
    #=====================================#
    #    Set up Publish/Subscribe Loop    #
    #=====================================#

    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        sub_cortex  = rospy.Subscriber('/cortex_raw' , Cortex, GetStates)
        blub = rospy.Subscriber('/trajectory' , Trajectories, GetTrajectory)
        rospy.spin()
        #r.sleep()


#!/usr/bin/env python

'''======================================================
        Trajectory Shaping AIAA SCITECH (26 MAY 2015)
   ======================================================'''

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
from ardrone_autonomy.msg import Navdata
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
time_past       = 0
start_time      = 0
I_Switch        = True

    #==================#
    #    Publishers    #
    #==================#

pub_ctrl        = rospy.Publisher('/controls', Controls, queue_size = 200)
pub_Range       = rospy.Publisher('/range', Float64, queue_size = 200)

    #=====================#
    #    Gain Matrices    #
    #=====================#

K = np.matrix([[0.45, 0, 0, 0.45, 0, 0, 0],[0, 0.45, 0, 0, 0.45, 0, 0],[ 0, 0, -0.6, 0, 0, -6.6, 0],[0, 0, 0, 0, 0, 0, 1]])

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

    #========================#
    #    Get Cortex States   #
    #========================#

def GetTrajectory(W):

    traj = W
    SWp_Controller(traj)

def GetStates(S):

    global states
    states = S

    #========================#
    #    Hows Controller    #
    #========================#

def SWp_Controller(traj):
    global states,PI, euler_max, max_yaw_rate, max_alt_rate, pub_ctrl, time_past,I_Switch,K,pub_Range

    Ctrl        = Controls()
    # Initiate Control Messages
    bodies = 1
    Ctrl.Obj = [Control()]*bodies
    Ctrl.header.stamp = states.header.stamp


    g = 9.81
    m = 0.450 

    #==============================#
    #   User defined Parameters    #
    #==============================#


    kv = 2
    V_veh = 0.1
    R_star = 0.4        
    Init_Acceleration = 0.2888
    K_psi = 3
    K_theta = 3

    if traj.Obj[0].name == "SWp":
        if states.Obj[0].visible:
            x = states.Obj[0].x
            y = states.Obj[0].y
            z = states.Obj[0].z
            xt = traj.Obj[0].x
            yt = traj.Obj[0].y
            zt = traj.Obj[0].z

       
            X = np.asmatrix(np.zeros((3,1)))
            X[0] = states.Obj[0].u #Vn
            X[1] = states.Obj[0].v #Ve
            X[2] = states.Obj[0].w #Vd
            V = np.linalg.norm(X)
            #V = sqrt(X[1,-1]*X[1,-1]+X[0,-1]*X[0,-1]+X[2,-1]*X[2,-1])
            psi_v   = atan2(X[1,-1],X[0,-1])
            theta_v = atan2(-X[2,-1],V) #X[2,-1] (+)ve or (-)ve, It is negative in matlab

            Range = sqrt((x-xt)*(x-xt)+(y-yt)*(y-yt)+(z-zt)*(z-zt))      
            Rxy = Range
            #Rxy   = sqrt((x-xt)*(x-xt)+(y-yt)*(y-yt))
            psi_l = atan2(yt-y, xt-x)
            theta_l = asin(-(zt-z)/Range) # (zt-z) is (negative in matlab)
            depsi = psi_l-psi_v
            dpsi = pi2pi(depsi)
            detheta = theta_l-theta_v
            dtheta = pi2pi(detheta)
            ah = K_psi*dpsi
            av = K_theta*dtheta
            thetadot = av/V
            psidot = ah/(V*cos(theta_v))

            vdot=kv*(V_veh-V)
     
            #acc_scale = 7
            #vel_scale = 1
            #if  X[0] > -V_veh and I_Switch:## X[0] > -0.1*vel_scale  and
            #    aax = -0.01*acc_scale
            #    aay = 0#0.01*acc_scale
            #else:
            #    aax=(vdot*cos(theta_v)*cos(psi_v))+ax
            #    aay=(vdot*cos(theta_v)*sin(psi_v))+ay
            aax=-(V*sin(theta_v)*cos(psi_v)*thetadot)-(V*cos(theta_v)*sin(psi_v)*psidot)+(vdot*cos(theta_v)*cos(psi_v)) #check thetadot sign
            aay=-(V*sin(theta_v)*sin(psi_v)*thetadot)+(V*cos(theta_v)*cos(psi_v)*psidot)+(vdot*cos(theta_v)*sin(psi_v))
               # I_Switch = False
            aaz = -V*cos(theta_v)*thetadot-g-vdot*sin(theta_v)

       #aaz = ((V_veh*cos(theta_v)*thetadot)+(vdot*sin(theta_v))) +g
       #print I_Switch

       #aax=(vdot*cos(theta_v)*cos(psi_v))+ax
       #aay=(vdot*cos(theta_v)*sin(psi_v))+ay
       #aaz = g+(vdot*sin(theta_v))+az      
       
            u=np.matrix([[aax],[aay],[aaz],[0]]) 
       #u = np.matrix([[0],[0],[9.81],[0]]) #negative values of x and y to move in positive direction
       #print aaz      
            #print u.T

       #==================================#
       #     Rotate to Vehicle 1 Frame    #
       #==================================#

            psi = states.Obj[0].psi*PI/180 #psi is negative in first quadrant
            rotZ = np.matrix([[cos(-psi), -sin(-psi), 0],[sin(-psi), cos(-psi), 0],[0, 0, 1]])
       #Cart = np.matrix([[-1, 0, 0],[0, -1, 0],[0, 0, 1]]) # fix x and y directions
            u[:-1] = rotZ*u[:-1]

       
        #print u

        #===================================#
        #     Normalize given the Thrust    #
        #===================================#

            T = sqrt(u[0,-1]*u[0,-1]+u[1,-1]*u[1,-1]+u[2,-1]*u[2,-1])
            u[:-1] = np.divide(u[:-1],T)
        #==================#
        #   Set Controls   #
        #==================#

            ctrl        = Control()
            ctrl.name   = states.Obj[0].name
            ctrl.phi    = asin(u[1,-1])/euler_max       
            ctrl.theta  = atan2(u[0,-1],u[2,-1])/euler_max
            ctrl.psi    = u[3,-1]/max_yaw_rate
            ctrl.T      = T*m
            Ctrl.Obj[0] = ctrl
            Ctrl.header = states.header       
            pub_ctrl.publish(Ctrl)
            pub_Range.publish(Rxy)



    else:
        if states.Obj[0].visible:
            X = np.asmatrix(np.zeros((7,1)))
            X[0] = traj.Obj[0].x-states.Obj[0].x
            X[1] = traj.Obj[0].y-states.Obj[0].y
            X[2] = traj.Obj[0].z-states.Obj[0].z
            X[3] = traj.Obj[0].xdot-states.Obj[0].u
            X[4] = traj.Obj[0].ydot-states.Obj[0].v
            X[5] = traj.Obj[0].zdot-states.Obj[0].w
            X[6] = pi2pi(traj.Obj[0].psi)-states.Obj[0].psi*PI/180


            Rxy = sqrt((X[0])*(X[0])+(X[1])*(X[1])+(X[2])*(X[2]))



            #============================================#
            #     Differential Flatness Control Input    #
            #============================================#

            # LQR input
            utilde = -K*X
            # required input
            u_r = np.matrix([[traj.Obj[0].xddot],[traj.Obj[0].yddot],[traj.Obj[0].zddot],[traj.Obj[0].psiddot]])
            u = utilde-u_r+np.matrix([[0],[0],[9.81],[0]])

            #==================================#
            #     Rotate to Vehicle 1 Frame    #
            #==================================#

            psi = states.Obj[0].psi*PI/180
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
       
            ctrl        = Control()
            ctrl.name   = states.Obj[0].name        
            ctrl.phi    = asin(u[1,-1])/euler_max        
            ctrl.theta  = atan2(u[0,-1],u[2,-1])/euler_max
            ctrl.psi    = u[3,-1]/max_yaw_rate
            ctrl.T      = T*m
            Ctrl.Obj[0] = ctrl
            Ctrl.header = states.header        
            pub_ctrl.publish(Ctrl)
            pub_Range.publish(Rxy)




    #===================#
    #       Main        #
    #===================#

if __name__=='__main__':
    import sys
    rospy.init_node('Hows_Controller')

    #=======================#
    #    quad parameters    #
    #=======================#

    euler_max    = float(rospy.get_param("euler_angle_max","0.349066")) #in radians
    max_yaw_rate = float(rospy.get_param("control_yaw",".3490659")) #in radians/sec
    max_alt_rate = float(rospy.get_param("control_vz_max","1000")) #in mm/sec


    #=====================================#
    #    Set up Publish/Subscribe Loop    #
    #=====================================#

    sub_cortex  = rospy.Subscriber('/cortex_raw' , Cortex, GetStates)
    blub = rospy.Subscriber('/trajectory' , Trajectories, GetTrajectory)
    rospy.spin()


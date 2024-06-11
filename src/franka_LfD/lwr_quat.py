"""
lwr_quat.py

Locally weighted regression for quaternion trajectories 
we learn using the angular velocities, then project back into quaternions
we know the inital orientation of course.
N.B: We assume we have both the quaternions and angular velocities as input

Author: Youssef Michel
Date: June 7, 2024
"""


#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import math  as math
import numpy as np 
import scipy as scip
from sklearn.linear_model import LinearRegression
import os
import matplotlib.pyplot as plt
from scipy.signal import filtfilt, butter
from lwr import lwr
from pyquaternion import Quaternion
from myQuaternion import myQuaternion

class lwr_quat(lwr): 
    def __init__(self,model_file = None , Des_traj_data= None ): 
        
        if model_file is not None:
            # Load model from file
            temp=np.genfromtxt(model_file) 
            self.Des_traj_quat= temp[:,:4] 
            self.Omega= temp[:, -3:] 

        elif Des_traj_data is not None:
            
            self.Des_traj_quat = Des_traj_data[:,:4] 
            self.Omega = Des_traj_data[:,-3: ]  
        else:
            # Default behavior if neither model_file nor matrix is provided
            raise ValueError("Either model_file or Data matrix must be provided !!!")

        self.Des_traj_quat = Des_traj_data
        order=4 
        cutoff_freq=5
        self.N_points= len(self.Des_traj_quat)

        print("Number of points:  ", self.N_points)
        
        b, a = butter(order, cutoff_freq / (0.5 * self.N_points), btype='low')

        self.Des_traj_quat=filtfilt(b,a,self.Des_traj_quat,axis=0) 
        self.Omega=filtfilt(b,a,self.Omega,axis=0) 
        self.Des_traj=self.Omega  #since LWR is performed on omega now
        self.N_models=8
            
        self.dt=0.001 
        self.Time_tot= self.N_points  *self.dt
        self.Time= np.linspace(0,self.Time_tot,self.N_points) 
        self.centers=np.linspace(0,self.Time_tot,self.N_models)

    def regression(self):
        
         sum= np.zeros( [self.N_points, self.Des_traj.shape[1] ] )
         
         for i in range(self.N_models):    
            sum= sum + self.W_k[i] @ self.X @ self.A_k[i]

        
         quat_curr = Quaternion(self.Des_traj_quat[0,:4]) 
         sim_traj = []
       

         for i in range( len(sum)):
          
            omega=sum[i,:]
            dt=0.001
            eta= dt*0.5* omega 
            quat_tmp= myQuaternion.exp_map(eta, quat_curr)
            quat_curr=quat_tmp
        
            quat_curr_vec=np.array([quat_curr.w, quat_curr.x, quat_curr.y, quat_curr.z])
            sim_traj.append(quat_curr_vec) 
                 #project using exp. map in UQ 
         
     
         return sim_traj





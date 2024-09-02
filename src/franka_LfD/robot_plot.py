"""
Plotting of the executed robot motions

Author: Youssef Michel
Date: June 7, 2024
"""

#!/usr/bin/env python
import rospy
import math  as math
import numpy as np 
import scipy as scip
from sklearn.linear_model import LinearRegression
import os
import matplotlib.pyplot as plt


if __name__ == '__main__': 

    catkin_ws_dir = os.path.expanduser("~/Codes/franka_ws") 
    N_skills=1 #fixed for now
    act_pos_file= catkin_ws_dir + "/src/franka_LfD/data/rob_pose_actual.txt" 
    act_pose=np.genfromtxt(act_pos_file)

    act_pos_file= catkin_ws_dir + "/src/franka_LfD/data/rob_pose_quat_actual.txt" 
    act_pose_quat=np.genfromtxt(act_pos_file)

    Des_traj=[]
    Des_traj_quat=[]
    
    for i in range(N_skills):
        file_name= catkin_ws_dir + "/src/franka_LfD/data/skill_" + str(i) +".txt"
        temp=np.genfromtxt(file_name)
        Des_traj.append(temp) 
  

        file_name= catkin_ws_dir + "/src/franka_LfD/data/skill_quat_" + str(i) +".txt"
        temp=np.genfromtxt(file_name)
        Des_traj_quat.append(temp) 
    
    Des_traj_tot = np.concatenate(Des_traj, axis=0)
    Des_traj_tot_quat = np.concatenate(Des_traj_quat, axis=0)

    plt.plot(act_pose[4700:,:3])
   
    plt.plot(Des_traj_tot[:,-3:],'--')
   
    plt.title("Executed act vs learnt (Pos)")
    plt.show()

    print(Des_traj_tot_quat[1,:4])
    print(act_pose_quat[1,:4])

    plt.plot(Des_traj_tot_quat[:,:4], '--')
    plt.plot(act_pose_quat[:,:4])

    print(Des_traj_tot_quat [-1 ,:4])
    plt.title("Executed act vs learnt (Quat)")
    plt.show()



#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import rospkg
import math  as math
import numpy as np 
import scipy as scip
from sklearn.linear_model import LinearRegression
import os
import matplotlib.pyplot as plt
from scipy.signal import filtfilt, butter
# from franka_LfD.lwr import lwr
from lwr import lwr
from lwr_quat import lwr_quat
from dmp import dmp 
from dmp_quat import dmp_quat
from pyquaternion import Quaternion
from myQuaternion import myQuaternion

class skill_learner_grasp: 
    def __init__(self,model_file,model_file_quat,model_file_gripper):

        if  not os.path.isdir(model_file):  
             raise FileNotFoundError(f"The translational trajectory directory does not exist.")
        
        if  not os.path.isdir(model_file_quat):  
             raise FileNotFoundError(f"The orientation trajectory directory does not exist.")

        if  not os.path.isdir(model_file_gripper):  
             raise FileNotFoundError(f"The gripper trajectory directory does not exist.")

        self.Des_traj= np.genfromtxt(model_file) 
        self.Des_traj_quat = np.genfromtxt(model_file_quat) 
        self.Des_traj_gripper = np.genfromtxt(model_file_gripper) 

        if len(self.Des_traj <100) or len(self.Des_traj_quat <100) or len(self.Des_traj_gripper<100): 
            raise ValueError("Invalid trajectory files")

        self.segment_trajectory()  
    

    def segment_trajectory(self):

        grip_width_traj=self.Des_traj_gripper[:,:1] 
        prev_gripper_state= grip_width_traj[0] 
        self.traj_segments=[]
        self.traj_segments_quat=[]
        indx_list =[0]
        indx_list_traj= [0]
        N_states=0 
        State_list= ['f']

        """
        g: grasp
        r: release 
        o: grasped
        f: released
        """

        # We assume a change of state is captured by the gripper width change

        for i in range(1,len(grip_width_traj)):

            curr_gripp_state= grip_width_traj[i]
            
            if(curr_gripp_state != prev_gripper_state):
                indx_list.append(i) 
                quat_curr= Quaternion(self.Des_traj_quat[i,:]) 
                quat_prev= Quaternion(self.Des_traj_quat[indx_list[N_states],:]) 
                log_diff= myQuaternion.log_map(quat_curr,quat_prev) 
                x_curr = self.Des_traj[i,:] 
                x_prev=  self.Des_traj[indx_list[N_states],:]

                if np.linalg.norm( x_curr-x_prev ) >0.0001 or np.linalg.norm(log_diff) >0.0001:

                    self.traj_segments.append(self.Des_traj[indx_list[N_states]:i,:])
                    self.traj_segments_quat.append(self.Des_traj_quat[indx_list[N_states]:i,:])
                    

                
                N_states +=1 

            prev_gripper_state=curr_gripp_state        

    

                

                
    def encodeSkills(self, Des_traj, Des_traj_quat):
        a=0 



        




if __name__ == '__main__': 

    catkin_ws_dir = os.path.expanduser("~/Codes/franka_ws") 
    
    model_file_quat= catkin_ws_dir + "/src/franka_LfD/data/rob_pose_quat_demo.txt" 
    model_file = catkin_ws_dir + "/src/franka_LfD/data/rob_pose_demo.txt" 
    model_file_gripper=  catkin_ws_dir + "/src/franka_LfD/data/gripper_state_demo.txt"  
    gripper_state=np.genfromtxt(model_file_gripper)
    plt.plot(gripper_state)
    plt.show() 
    skill_learner_=skill_learner_grasp(model_file,model_file_quat,model_file_gripper)
    skill_learner_.learn_skill("DMP")


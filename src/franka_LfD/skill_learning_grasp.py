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
from dmp import dmp, interpolate_traj
from dmp_quat import dmp_quat
from pyquaternion import Quaternion
from myQuaternion import myQuaternion
import yaml 

class skill_learner_grasp: 
    def __init__(self,model_file,model_file_quat,model_file_gripper,dt=0.001):

        # if  not os.path.isdir(model_file):  
        #      raise FileNotFoundError(f"The translational trajectory directory does not exist.")
        
        # if  not os.path.isdir(model_file_quat):  
        #      raise FileNotFoundError(f"The orientation trajectory directory does not exist.")

        # if  not os.path.isdir(model_file_gripper):  
        #      raise FileNotFoundError(f"The gripper trajectory directory does not exist.")
        
        temp=np.genfromtxt(model_file)
        self.Des_traj=temp[:,:3] 

        order=2
        cutoff_freq=4
        n_points = len(self.Des_traj) 
        b, a = butter(order, cutoff_freq / (0.5 * n_points), btype='low')
        TrajTemp=filtfilt(b,a,self.Des_traj,axis=0) 
        Traj_dot=np.diff(TrajTemp,axis=0) /dt
        Traj_dot= interpolate_traj(Traj_dot,n_points)
        self.Des_traj= np.hstack((self.Des_traj, Traj_dot))
        tempQ = np.genfromtxt(model_file_quat)
        self.Des_traj_quat=tempQ[:,:] 
        self.Des_traj_gripper=np.genfromtxt(model_file_gripper)
        self.Des_traj_quat[:,-3:]=filtfilt(b,a,self.Des_traj_quat[:,-3:],axis=0) 

        plt.figure()
        plt.plot(self.Des_traj[:,-3:])
        plt.figure()
        plt.plot(self.Des_traj_gripper)
        plt.show()
      
        # if len(self.Des_traj <100) or len(self.Des_traj_quat <100) or len(self.Des_traj_gripper<100): 
        #     raise ValueError("Invalid trajectory files")

        self.segment_trajectory()  
        self.encodeSkills()
        self.saveSkills()
    

    def segment_trajectory(self):

        grip_width_traj=self.Des_traj_gripper[:,0] 
        prev_gripper_state= grip_width_traj[0] 
        self.traj_segments=[]
        self.traj_segments_quat=[]
        indx_list =[0]
        indx_list_traj= [0]
        N_states_mov=0 # number of movement states
        self.State_list= []

        """
        m: move
        o: open
        c: close
        """

        # We assume a change of state is captured by the gripper width change
        for i in range(1,len(grip_width_traj)):

            curr_gripp_state= grip_width_traj[i]
            
            if(curr_gripp_state != prev_gripper_state):
                indx_list.append(i) 
                quat_curr= Quaternion(self.Des_traj_quat[i,:4]) 
                quat_prev= Quaternion(self.Des_traj_quat[indx_list[N_states_mov],:4]) 
                log_diff= myQuaternion.log_map(quat_curr,quat_prev) 
                x_curr = self.Des_traj[i,:3] 
                x_prev=  self.Des_traj[indx_list[N_states_mov],:3]

                if np.linalg.norm( x_curr-x_prev ) >0.001 or np.linalg.norm(log_diff) >0.001:

                    self.traj_segments.append(self.Des_traj[indx_list[N_states_mov]:i,:])
                    self.traj_segments_quat.append(self.Des_traj_quat[indx_list[N_states_mov]:i,:])
                    self.State_list.append('m')
                    if(curr_gripp_state>prev_gripper_state):
                        self.State_list.append('o')
                    else:
                        self.State_list.append('c')
                
                else: 
                    
                    if(curr_gripp_state>prev_gripper_state):
                        self.State_list.append('o')
                    else:
                        self.State_list.append('c')

                N_states_mov +=1 
            

            prev_gripper_state=curr_gripp_state 
        
        print("states: ", self.State_list)


                
    def encodeSkills(self):
        
        N_mov_prim= len(self.traj_segments)
        self.traj_segments_quat_learnt=[] 
        self.traj_segments_learnt=[]

        for i in range(N_mov_prim): 
            
            curr_traj= self.traj_segments[i] 
            curr_traj_quat=self.traj_segments_quat[i]
            print(curr_traj_quat[:,-3:].shape)
            r= self.obtain_relevant_motion(curr_traj[:,-3:], curr_traj_quat[:,-3:])  

            # Do additional segmentation to learn the actual motion
            if r == 'q': 
               indx_st,indx_end = segment_normbased(curr_traj_quat[:,-3:])
            else:
               indx_st,indx_end =  segment_normbased( curr_traj[:,-3:])
            

            print("indices: ", indx_st,indx_end)
            N_red=200 
            N_org= len(curr_traj[indx_st:indx_end,:3])
            temp=interpolate_traj(curr_traj[indx_st:indx_end,:3],N_red)
            MyDmPLearner = lwr(None,curr_traj[indx_st:indx_end,:3])
            MyDmPLearner.learn_lwr()
            Learnt_traj= MyDmPLearner.regression()
            # Learnt_traj=interpolate_traj(Learnt_traj_tmp,N_org)
            self.traj_segments_learnt.append(Learnt_traj)

            MyDmPLearnerQuat = lwr(None,curr_traj_quat[indx_st:indx_end,:4])
            MyDmPLearnerQuat.learn_lwr()
            Learnt_traj_quat= MyDmPLearnerQuat.regression()  
            self.traj_segments_quat_learnt.append(Learnt_traj_quat)
            plt.figure()
            plt.plot(Learnt_traj_quat)
            plt.plot(curr_traj_quat[indx_st:indx_end,:4],'--')
            
        plt.show()
    
    def saveSkills(self): 
        packpath=os.path.expanduser("~/Codes/franka_ws") 

        for i in range(len(self.traj_segments_learnt)):
            traj_learnt= self.traj_segments_learnt[i] 
            traj_learnt_quat= self.traj_segments_quat_learnt[i]  
            file_path=packpath+ "/src/franka_LfD/data/skill_"+str(i) +".txt" 
            file_path_quat=packpath+ "/src/franka_LfD/data/skill_quat_"+str(i) +".txt" 
            with open(file_path, 'w') as f:
                   for row in traj_learnt:
                      f.write(' '.join(map(str, row)) + '\n')
            
            with open(file_path, 'w') as f:
                   for row in traj_learnt_quat:
                      f.write(' '.join(map(str, row)) + '\n')
        
        file_path_state = packpath+ "/src/franka_LfD/data/state_list.txt" 
        with open(file_path_state, 'w') as file:
            for char in self.State_list:
                 file.write(char + '\n')

        N_states= len(self.State_list)
        params = {
        'N_states': N_states
        }

        file_path_yaml = packpath+ "/src/franka_LfD/config/learning_params.yaml" 
        with open(file_path_yaml, 'w') as file:
                yaml.dump(params, file, default_flow_style=False)

    def obtain_relevant_motion(self,vel_trans_traj, vel_quat_traj):
        """
        For segmentation purposes, we want to know whether this is
        mainly an orientational or translational motion 
        """
        if  np.mean(np.linalg.norm(vel_quat_traj,axis=1)) > np.mean(np.linalg.norm(vel_trans_traj,axis=1)):
            return 'q'
        else:
            return 't'
          
    
def segment_normbased(Traj):
    Traj_norm= np.linalg.norm(Traj,axis=1)
    indx_start = 0 
    indx_end=len(Traj_norm) -1
    started_flag= False

    for i in range(len(Traj_norm)): 
        
        if( np.mean(Traj_norm[i:i+50]) >0.05 ) and  started_flag is False: 
            indx_start= i 
            started_flag= True

        if started_flag and  np.mean(Traj_norm[i:i+50]) <0.02 :
            indx_end= max(len(Traj_norm), i+300) 

    return indx_start, indx_end  




                




                    
                    


            



        


        





        




if __name__ == '__main__': 

    catkin_ws_dir = os.path.expanduser("~/Codes/franka_ws") 
    print(catkin_ws_dir)
    
    model_file_quat= catkin_ws_dir + "/src/franka_LfD/data/rob_pose_quat_demo.txt" 
    model_file = catkin_ws_dir + "/src/franka_LfD/data/rob_pose_demo.txt" 
    model_file_gripper=  catkin_ws_dir + "/src/franka_LfD/data/gripper_state_demo2.txt"  
    gripper_state=np.genfromtxt(model_file_gripper)




    skill_learner_=skill_learner_grasp(model_file,model_file_quat,model_file_gripper)



#!/usr/bin/env python
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


class skill_learner: 
    def __init__(self,model_file,model_file_quat): 
        temp=np.genfromtxt(model_file)  
        self.Des_traj=temp[:,:3]
        self.Des_traj_quat = np.genfromtxt(model_file_quat) 
        self.dt =0.001
        order = 4
        cutoff_freq = 4  # in Hz
        self.n_points=len(self.Des_traj) 

        self.b, self.a = butter(order, cutoff_freq / (0.5 * self.n_points), btype='low')
        self.Des_traj[:,0]=filtfilt(self.b,self.a,self.Des_traj[:,0])
        self.Des_traj[:,1]=filtfilt(self.b,self.a,self.Des_traj[:,1])
        self.Des_traj[:,2]=filtfilt(self.b,self.a,self.Des_traj[:,2])
        self.Des_traj_dot= np.diff(self.Des_traj,axis=0) /self.dt 
        plt.plot(self.Des_traj_dot)
        plt.show()

        self.Des_traj_dot_norm=np.abs(np.mean(self.Des_traj_dot,axis=1) )
        self.vel_thresh=0.01
        self.n_segments=1 

        plt.plot(self.Des_traj)
        plt.figure()
        plt.plot((self.Des_traj_dot_norm))
        plt.show()
    
    def segment_traj(self): 
        self.indices_start = []
        self.indices_end   = []
        flag_new=True 
        last_ind=0 
        moved_flag=False
        
        for i in range(len(self.Des_traj_dot_norm)): 
            phase_dur= (i-last_ind)*self.dt

            if( np.mean(self.Des_traj_dot_norm[i:i+100]) >self.vel_thresh and flag_new==True  ): 
                self.indices_start.append(i) 
                flag_new=False 
                last_ind=i 
                moved_flag=False

            if(flag_new==False): 
                if(np.mean(self.Des_traj_dot_norm[i:i+100])>self.vel_thresh):
                   moved_flag=True 
                   
            if( np.mean(self.Des_traj_dot_norm[i:i+100]) <self.vel_thresh and flag_new==False and phase_dur>1 and moved_flag):
                self.indices_end.append(i)
                flag_new=True  
               
    
        self.traj_segments=[]
        self.traj_segments_quat= [] 

        for i in range (len(self.indices_start)) : 

            temp_traj=self.Des_traj[self.indices_start[i] : self.indices_end[i],:] 
            temp_traj_quat = self.Des_traj_quat[self.indices_start[i] : self.indices_end[i],:] 
            self.traj_segments.append(temp_traj)
            self.traj_segments_quat.append(temp_traj_quat)


    
    def encode_skills(self,mode):

        self.learnt_traj= [] 
        print("Number of segments: ", len(self.traj_segments))
        
        for i in range (len(self.traj_segments)): 
            curr_traj=self.traj_segments[i] 
            curr_traj_quat=self.traj_segments_quat[i]
            
            if mode == "LWR":
                lwr_learner = lwr(Des_traj_data=curr_traj) 
                lwr_learner.learn_lwr()
                traj_learnt=lwr_learner.regression()
                lwr_learner_quat=lwr_quat(Des_traj_data=curr_traj_quat)
                lwr_learner_quat.learn_lwr() 
                traj_learnt_quat=lwr_learner_quat.regression()
                

            else:
                DMP_learner= dmp(Des_traj_data=curr_traj)
                DMP_learner_quat= dmp_quat(Des_traj_data=curr_traj_quat) 
                DMP_learner.learn_dmp() 
                traj_learnt= DMP_learner.simulate_dmp_dynamics() 
                DMP_learner_quat.learn_dmp()
                traj_learnt_quat = DMP_learner_quat.simulate_dmp_dynamics()

            
            plt.plot(traj_learnt) 
            plt.plot(curr_traj,'--')
            title_pos= mode + " Learnt vs Actual Position, Skill: " + str(i)
            plt.title(title_pos)
            plt.show()

            plt.figure()
            plt.plot(traj_learnt_quat)
            plt.plot(curr_traj_quat[:,:4],'--')
            title_quat= mode + " Learnt vs Actual Quat, Skill: " + str(i)
            plt.title(title_quat)
            plt.show()


            # rospack = rospkg.RosPack()
            # rospack.list() 
            packpath=os.path.expanduser("~/Codes/franka_ws") 
            file_path=packpath+ "/src/franka_LfD/data/skill_"+str(i) +".txt" 
            file_path_quat=packpath+ "/src/franka_LfD/data/skill_quat_"+str(i) +".txt" 
            with open(file_path, 'w') as f:
                   for row in traj_learnt:
                      f.write(' '.join(map(str, row)) + '\n')

            # with open(file_path, "w") as f:
            #    for line in np.matrix(traj_learnt):
            #      np.savetxt(f, line, fmt='%.3f')

            # np.savetxt(file_path, np.array(traj_learnt), fmt='%.3f')
            np.savetxt(file_path_quat, traj_learnt_quat, fmt='%.3f')
            test=np.genfromtxt(file_path)
 
            self.learnt_traj.append(traj_learnt)

        return self.learnt_traj
    
    def learn_skill(self, mode): 
        self.segment_traj() 
        self.encode_skills(mode)
        
        plt.show()

        print("indices start: ", self.indices_start) 
        print("indices end: ", self.indices_end) 

    

if __name__ == '__main__': 

    catkin_ws_dir = os.path.expanduser("~/Codes/franka_ws") 
    
    model_file_quat= catkin_ws_dir + "/src/franka_LfD/data/rob_pose_quat_demo.txt" 
    model_file = catkin_ws_dir + "/src/franka_LfD/data/rob_pose_demo.txt" 
    model_file_gripper=  catkin_ws_dir + "/src/franka_LfD/data/gripper_state_demo.txt"  
    gripper_state=np.genfromtxt(model_file_gripper)
    plt.plot(gripper_state)
    plt.show() 


    skill_learner_=skill_learner(model_file,model_file_quat)
    skill_learner_.learn_skill("DMP")


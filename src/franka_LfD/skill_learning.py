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

class skill_learner: 
    def __init__(self,model_file): 
        self.Des_traj=np.genfromtxt(model_file)  
        self.dt =0.001
        order = 4
        cutoff_freq = 4  # in Hz
        self.n_points=len(self.Des_traj)

        self.b, self.a = butter(order, cutoff_freq / (0.5 * self.n_points), btype='low')
        self.Des_traj[:,0]=filtfilt(self.b,self.a,self.Des_traj[:,0])
        self.Des_traj[:,1]=filtfilt(self.b,self.a,self.Des_traj[:,1])
        self.Des_traj[:,2]=filtfilt(self.b,self.a,self.Des_traj[:,2])
        self.Des_traj_dot= np.diff(self.Des_traj,axis=0) /self.dt
        self.Des_traj_dot_norm=np.linalg.norm(self.Des_traj_dot,axis=1) 
        self.vel_thresh=0.01 
        self.n_segments=1 

        plt.plot(self.Des_traj)
        plt.figure()
        plt.plot(self.Des_traj_dot_norm)
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
                if(np.mean(self.Des_traj_dot_norm[i:i+100])>0.03):
                   moved_flag=True 
            if( np.mean(self.Des_traj_dot_norm[i:i+100]) <self.vel_thresh and flag_new==False and phase_dur>1 and moved_flag):
                self.indices_end.append(i)
                flag_new=True  
               # moved_flag=False
        
        self.traj_segments=[]
        print(self.indices_start)
        print(self.indices_end)
        for i in range (len(self.indices_start)) : 

            temp_traj=self.Des_traj[self.indices_start[i] : self.indices_end[i],:] 
            self.traj_segments.append(temp_traj)


    
    def encode_skills(self):

        self.learnt_traj= []
        for i in range (len(self.traj_segments)): 
            curr_traj=self.traj_segments[i] 
            
            print(len(curr_traj))

            lwr_learner = lwr(Des_traj_data=curr_traj) 
            lwr_learner.learn_lwr()
            traj_learnt=lwr_learner.regression()
            rospack = rospkg.RosPack()
            rospack.list() 
           # packpath= rospack.get_path('franka_LfD') 
            packpath="/home/dhrikarl/Codes/franka_ws/src/franka_LfD"
            file_path=packpath+ "/data/skill_"+str(i) +".txt" 
            np.savetxt(file_path, traj_learnt, fmt='%.3f')
            plt.plot(curr_traj)
            plt.plot(traj_learnt,'--')
            plt.show()
            self.learnt_traj.append(traj_learnt)

        return self.learnt_traj
    
    def learn_skill(self): 
        self.segment_traj() 
        self.encode_skills( )
        
        plt.show()

        print("indices start: ", self.indices_start) 
        print("indices end: ", self.indices_end) 

    

if __name__ == '__main__': 

    model_file= "/home/dhrikarl/Codes/franka_ws/src/franka_LfD/data/rob_pose_demo.txt" 
    skill_learner_=skill_learner(model_file)
    skill_learner_.learn_skill()


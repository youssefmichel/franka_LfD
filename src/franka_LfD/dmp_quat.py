import rospy
from geometry_msgs.msg import Twist
import math  as math
import numpy as np 
import scipy as scip
from sklearn.linear_model import LinearRegression
import os
import matplotlib.pyplot as plt
from scipy.signal import filtfilt, butter
from collections import namedtuple
from dataclasses import dataclass
from pyquaternion import Quaternion
from myQuaternion import myQuaternion
from dmp import gauss_pdf, canonical_dynamics, interpolate_traj

class dmp_params_quat:
    dt: float= 0.001
    n_models: int= 6
    alpha: float= 0.1
    kp: float=0.0001 #spring part affects learning @TODO: Check
    kd: float= 2*1.0* np.sqrt(kp)
    


class dmp_quat: 
    def __init__(self,model_file = None , Des_traj_data= None ): 
       
        if model_file is not None:
            
            temp=np.genfromtxt(model_file) 
            self.Des_traj= temp[3000:7000,:4] 
            self.Omega= temp[3000:7000, -3:] 
            
        elif Des_traj_data is not None:
           
            self.Des_traj = Des_traj_data[:,:4] 
            self.Omega = Des_traj_data[:,-3: ]    
            
        
        else:
            raise ValueError("Either model_file or Data matrix must be provided !!!")


        
        self.dmp_params=dmp_params_quat()  
        order=2
        cutoff_freq=15
        self.n_points=len(self.Des_traj)
        # self.n_points=2000 

        self.time_end= self.n_points* self.dmp_params.dt 
        b, a = butter(order, cutoff_freq / (0.5 * self.n_points), btype='low')

        self.Des_traj=filtfilt(b,a,self.Des_traj,axis=0) 
        self.Des_traj= interpolate_traj(self.Des_traj,self.n_points)
        self.Omega= interpolate_traj(self.Omega,self.n_points)
        self.Omega=filtfilt(b,a,self.Omega,axis=0) 
      
        self.Omega_dot= np.diff(self.Omega,axis=0) /self.dmp_params.dt 
        self.Omega_dot =  interpolate_traj(self.Omega_dot,self.n_points)
        self.Omega_dot=filtfilt(b,a,self.Omega_dot,axis=0) 

        self.quat_goal=Quaternion(self.Des_traj[self.n_points-1,:4]) 
       
        self.decay=canonical_dynamics(self.n_points,self.dmp_params.alpha,self.dmp_params.dt) 
     

        

    def learn_dmp(self): 
        self.Mu = np.linspace(0, self.time_end, self.dmp_params.n_models) 
        self.sigma=0.0002 
        self.build_regressors() 
        H = np.zeros( (self.dmp_params.n_models, self.n_points) )

        for i in range(self.dmp_params.n_models):
            sum=0 
            for j in range(self.n_points):
                temp= gauss_pdf(self.decay[j], self.sigma, self.Mu[i]) 
                H[i,j]=temp
                sum=sum+temp 

            H[i,:]= H[i,:] /sum 
        
        X=np.ones([self.n_points,1]) 
       

        self.Muf_list= []
        H_inv= np.linalg.pinv(H)
        Y= self.data_dmp
        self.currF= self.data_dmp.T @ H_inv 
        self.currF = self.currF @ H 



    def simulate_dmp_dynamics(self):
        quat_curr= Quaternion(self.Des_traj[0,:]) 

     
        omega=self.Omega[0,:]*1
        sim_traj=[]
        ref_acc_list=[] 
    
        quat_diff= myQuaternion.log_map(self.quat_goal, quat_curr)
        flag=True
   
        
        for i in range(self.n_points): 
   
            quat_diff= myQuaternion.log_map( quat_curr, self.quat_goal )

            ref_acc= self.dmp_params.kp* quat_diff - self.dmp_params.kd *omega + 1* self.currF[:,i] *self.decay[i]
     
            omega=omega + self.dmp_params.dt* ref_acc 

            eta= self.dmp_params.dt*0.5* omega 
            quat_tmp= myQuaternion.exp_map(eta, quat_curr)
        
            quat_curr=quat_tmp
            
            quat_curr_vec=np.array([quat_curr.w, quat_curr.x, quat_curr.y, quat_curr.z])
            sim_traj.append(quat_curr_vec) 
            ref_acc_list.append(ref_acc)
        
        
        sim_res=np.array(sim_traj) 
 
        plt.plot(sim_traj, label='DMP') 
        plt.plot(self.Des_traj,'--', label = 'Actual')
        plt.legend()
        plt.title('DMP Learning, Unit Quaternions')
        plt.grid(True)
        plt.show()

     

        return sim_res

        
    
    def build_regressors(self): 
        data_dmp_list=[] 
       

        for i in range(self.n_points): 
            quat_curr=Quaternion(self.Des_traj[i,:]) 
        
            quat_diff= myQuaternion.log_map( quat_curr, self.quat_goal )
            quat_diff_vec=quat_diff

            Y= self.Omega_dot[i,:] - self.dmp_params.kp  * quat_diff_vec
            + self.dmp_params.kd *self.Omega[i,:] 
           
            Y=Y/max(self.decay[i],0.01)
            data_dmp_list.append(Y) 
         
        self.data_dmp=np.array(data_dmp_list)



        

if __name__ == '__main__': 
    catkin_ws_dir = os.path.expanduser("~/Codes/franka_ws") 
    
    model_file= catkin_ws_dir + "/src/franka_LfD/data/rob_pose_quat_demo.txt" 
    
    skill_learner=dmp_quat(model_file)
    skill_learner.learn_dmp()
    skill_learner.simulate_dmp_dynamics()

  
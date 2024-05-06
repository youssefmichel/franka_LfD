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


class dmp_params:
    dt: float= 0.01 
    n_models: int= 4 
    alpha: float= 1
    kp: float=50 
    kd: float= np.sqrt(2*kp)


class dmp: 
    def __init__(self,model_file = None , Des_traj_data= None ): 
        
        if model_file is not None:
            # Load model from file
            self.Des_traj=np.genfromtxt(model_file) 
        elif Des_traj_data is not None:
            # Use the provided matrix
            self.Des_traj = Des_traj_data
        else:
            raise ValueError("Either model_file or Data matrix must be provided !!!")

        self.dmp_params=dmp_params()  
        order=2
        cutoff_freq=9
        self.n_points=len(self.Des_traj)
        self.time_end= self.n_points* self.dmp_params.dt 

     
        b, a = butter(order, cutoff_freq / (0.5 * self.n_points), btype='low')

        self.Des_traj=filtfilt(b,a,self.Des_traj,axis=0) 

        self.Des_traj_dot=np.diff(self.Des_traj,axis=0) /self.dmp_params.dt
        self.Des_traj_dot=filtfilt(b,a,self.Des_traj_dot,axis=0) 

        self.Des_traj_ddot=np.diff(self.Des_traj_dot,axis=0)/self.dmp_params.dt 
        self.Des_traj_ddot=filtfilt(b,a,self.Des_traj_ddot,axis=0) 

        self.n_points=1000 
        self.Des_traj= interpolate_traj(self.Des_traj,self.n_points)
        self.Des_traj_dot=interpolate_traj(self.Des_traj_dot,self.n_points)
        self.Des_traj_ddot= interpolate_traj(self.Des_traj_ddot,self.n_points)

        self.decay= canonical_dynamics(self.n_points,self.dmp_params.alpha,self.dmp_params.dt)
        plt.plot(self.decay) 
        plt.show()

    def learn_dmp(self): 

        self.Mu = np.linspace(0, self.time_end, self.dmp_params.n_models) 
        self.sigma=0.0002 
        self.data_dmp_list = [] 
        self.goal= self.Des_traj[self.n_points-1,: ]
        data_dmp_list=[]

        for i in range(self.n_points): 
            temp= self.Des_traj_ddot[i,:] - (self.goal - self.Des_traj[i,:]) * self.dmp_params.kp + self.Des_traj_dot[i,:]*self.dmp_params.kd
            
            temp = temp/self.decay[i] 
            data_dmp_list.append( temp ) 
        
        data_dmp=np.array(data_dmp_list)
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
        Y= data_dmp
        print( Y.shape)
        self.currF= data_dmp.T @ H_inv 
        self.currF = self.currF @ H 
        print(self.currF)



        plt.plot(H.transpose())
        plt.show()

    def simulate_dmp_dynamics(self): 
        
        x= self.Des_traj[0,:]
        x_dot=x*0 
        sim_traj=[]
        for i in range(self.n_points): 
            ref_acc= self.dmp_params.kp* (self.goal - x ) - self.dmp_params.kd * x_dot + self.currF[:,i] *self.decay[i]
            x_dot=x_dot + self.dmp_params.dt* ref_acc 
            x = x + self.dmp_params.dt * x_dot 
            sim_traj.append(x) 
        
        sim_res=np.array(sim_traj) 
        plt.plot(sim_res[:,0],sim_res[:,1],'--')
        plt.plot(self.Des_traj[:,0],self.Des_traj[:,1])
        plt.show()
        return sim_res



def interpolate_traj(x_traj,n_points):

        t_old= np.linspace(0,1, len(x_traj))
        f_spline = scip.interpolate.make_interp_spline(t_old, x_traj) 
        t_new=np.linspace(0,1,n_points) 
        x_intp=f_spline(t_new)
        return x_intp
    
    # def canonical_dynamics(self):
    #     self.decay=[]
    #     for i in range(self.n_points):
    #         self.decay.append( np.exp(-self.dmp_params.alpha* (i)*self.dmp_params.dt)) 
    #     return self.decay 
    
def canonical_dynamics(n_points,alpha,dt):
        decay=[]
        for i in range(n_points):
           decay.append( np.exp(-alpha* (i)*dt)) 

        return decay 

def gauss_pdf(x,h_i,c_i): 
        return math.exp(-h_i * math.pow((x-c_i),2))  


     
       # plt.plot(self.Des_traj_dot)
       # plt.show()
    
if __name__ == '__main__': 

    model_file= "/home/dhrikarl/Codes/franka_ws/src/franka_LfD/data/rob_pose_actual.txt" 
    skill_learner_=dmp(model_file)
    skill_learner_.learn_dmp()
    skill_learner_.simulate_dmp_dynamics()


        

        

        

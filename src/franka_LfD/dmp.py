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
    dt: float= 0.001 
    n_models: int= 8
    alpha: float= 4
    kp: float=100
    kd: float= 1*np.sqrt(2*kp)



class dmp: 
    def __init__(self, model_file = None , Des_traj_data= None ): 
        
        if model_file is not None:
            self.Des_traj=np.genfromtxt(model_file) 

        elif Des_traj_data is not None:
            self.Des_traj = Des_traj_data
        else:
            raise ValueError("Either model_file or Data matrix must be provided !!!")

        self.dmp_params=dmp_params()  
        order=2
        cutoff_freq=3
        self.n_points=len(self.Des_traj)
        self.time_end= self.n_points* self.dmp_params.dt 
        self.tau = self.time_end 
        
        

        b, a = butter(order, cutoff_freq / (0.5 * self.n_points), btype='low')
        self.Des_traj=filtfilt(b,a,self.Des_traj,axis=0) 

        self.Des_traj_dot=np.diff(self.Des_traj,axis=0) /self.dmp_params.dt
        self.Des_traj_dot=filtfilt(b,a,self.Des_traj_dot,axis=0) 

        self.Des_traj_ddot=np.diff(self.Des_traj_dot,axis=0)/self.dmp_params.dt 
        self.Des_traj_ddot=filtfilt(b,a,self.Des_traj_ddot,axis=0) 


        self.Des_traj= interpolate_traj(self.Des_traj,self.n_points)
        self.Des_traj_dot=interpolate_traj(self.Des_traj_dot,self.n_points)
        self.Des_traj_ddot= interpolate_traj(self.Des_traj_ddot,self.n_points)

        self.decay= canonical_dynamics(self.n_points,self.dmp_params.alpha,self.dmp_params.dt,self.tau)
        plt.plot(self.decay)
        plt.title("decay DMP") 
        plt.show()

    def learn_dmp(self): 

        self.Mu = np.linspace(0, self.time_end, self.dmp_params.n_models) 
        self.sigma=0.001
        self.data_dmp_list = [] 
        self.goal= self.Des_traj[self.n_points-1,: ]
        data_dmp_list=[]

        for i in range(self.n_points): 
            temp= pow(self.tau,2)* self.Des_traj_ddot[i,:] - (self.goal - self.Des_traj[i,:]) * self.dmp_params.kp + self.tau*self.Des_traj_dot[i,:]*self.dmp_params.kd
            data_dmp_list.append( temp ) 
        
        data_dmp=np.array(data_dmp_list)
        H= np.zeros(( self.n_points, self.dmp_params.n_models) )

        for i in range(self.n_points):
            sum = 0 
            for j in range(self.dmp_params.n_models):
                temp= gauss_pdf(self.decay[i], self.sigma, self.Mu[j]) 
                sum = sum + temp 
                H[i,j] = temp * self.decay[i]

            H[i,:] =  H[i,:] /sum 


        H_inv= np.linalg.pinv(H)
        W=   H_inv @ data_dmp
        self.currF = H @ W 
        
        plt.plot(self.currF) 
        plt.plot(data_dmp,'--')
        plt.show()
        print(self.currF[1,:])
     
      
    
      
    def simulate_dmp_dynamics(self): 
        
        y= self.Des_traj[0,:]
        z=0*y
        sim_traj=[]

        for i in range(self.n_points+300):

            if(i<self.n_points-2):
               z_dot= self.dmp_params.kp* (self.goal - y ) - self.dmp_params.kd * z+ 1*self.currF[i,:] 
            else: 
               z_dot= self.dmp_params.kp* (self.goal - y ) - self.dmp_params.kd * z

            z_dot=z_dot/self.tau 
            z= z + z_dot *self.dmp_params.dt 
            y_dot= z/self.tau 
            y = y+ self.dmp_params.dt *y_dot 
            sim_traj.append(y) 
        
        sim_res=np.array(sim_traj) 
     

        plt.plot(sim_res, label='DMP') 
        plt.plot(self.Des_traj,'--', label = 'Actual')
        plt.legend()
        plt.title('DMP Learning')
        plt.grid(True)
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
    
def canonical_dynamics(n_points,alpha,dt,tau=1):
        decay=[]
        for i in range(n_points):
           decay.append( np.exp(-alpha* (i)*dt /tau )) 

        return decay 

def gauss_pdf(x,h_i,c_i): 
        return math.exp(-h_i * math.pow((x-c_i),2))  


     
       # plt.plot(self.Des_traj_dot)
       # plt.show()
    
if __name__ == '__main__': 

    catkin_ws_dir = os.path.expanduser("~/Codes/franka_ws") 
    model_file= catkin_ws_dir + "/src/franka_LfD/data/rob_pose_demo.txt" 
    Des_tra_tot=np.genfromtxt(model_file) 
    Des_traj= Des_tra_tot[3700:6500,:3]
    skill_learner_=dmp(None,Des_traj)
    skill_learner_.learn_dmp()
    skill_learner_.simulate_dmp_dynamics()


        

        

        

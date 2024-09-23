import math  as math
import numpy as np 
import scipy as scip

import matplotlib.pyplot as plt

from pyquaternion import Quaternion


class myQuaternion(Quaternion):

    @classmethod
    def exp_map(cls,eta,q_inp):
        
        q= Quaternion(q_inp) 

        eta_norm=np.linalg.norm(eta) 
        eta_vec  = eta / eta_norm 

        if (eta_norm < 1e-9):
            # return Quaternion(1,0,0,0) 
            return q 

        else: 

            q_temp= Quaternion(scalar= np.cos(eta_norm) , vector= np.sin(eta_norm)*eta_vec)
            
            return q_temp *q 
            # return q_temp 
        
    @classmethod
    def log_map(cls, q_1, q_2 ): 
        q_1_= Quaternion(q_1) 
        q_2_=Quaternion(q_2) 
        q= q_2_ * q_1_.conjugate

        u= q.vector 
        v= q.scalar 
        if(v>1):
            v=1.0 
        if v<-1: 
            v=-1.0

        if (np.linalg.norm(u)>1e-4):
            scale=1/ np.linalg.norm(u) 
   

            return np.arccos(v) * u *scale
        else: 
            return np.zeros(3) 
    
    @classmethod
    def quat_2_omega(Q,dt):
        N=len(Q) 
        t = np.linspace(dt, N*dt , N ) 

        for i in range(N): 
            q_curr= Quaternion( Q[i,0] , Q[i,1] , Q[i,2] , Q[i,3]  )
            tmp = myQuaternion.norm()

    # differentiate a quaternion trajectory 

    

if __name__ == '__main__':  

     q1= myQuaternion(    0.8046  , -0.4391  , -0.0030  , -0.3999) 
     q2= myQuaternion(1,0,0,0) 

     print("log", myQuaternion.log_map(q2,q1))
     omega = np.array([0.006, -0.008, 0.0196])
     eta = -0.5  * omega
     q_res=myQuaternion.exp_map(eta,q1)  
     print("Exg",q_res)



        

        



       
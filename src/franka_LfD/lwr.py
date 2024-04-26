#!/usr/bin/env python
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

import math  as math
import numpy as np 
import scipy as scip
from sklearn.linear_model import LinearRegression
import os
import matplotlib.pyplot as plt
from scipy.signal import filtfilt, butter


class lwr: 
    def __init__(self,model_file = None , Des_traj_data= None ): 
        
        if model_file is not None:
            # Load model from file
            self.Des_traj=np.genfromtxt(model_file) 
        elif Des_traj_data is not None:
            # Use the provided matrix
            self.Des_traj = Des_traj_data
        else:
            # Default behavior if neither model_file nor matrix is provided
            raise ValueError("Either model_file or Data matrix must be provided !!!")

        self.Des_traj = Des_traj_data
        order=4 
        cutoff_freq=5
        self.N_points= len(self.Des_traj)

        print("Number of points:  ", self.N_points)
        
        self.b, self.a = butter(order, cutoff_freq / (0.5 * self.N_points), btype='low')

        self.Des_traj[:,0]=filtfilt(self.b,self.a,self.Des_traj[:,0])
        self.Des_traj[:,1]=filtfilt(self.b,self.a,self.Des_traj[:,1])

    
        self.dt=0.005 
        

        self.Time_tot= self.N_points  *self.dt


        self.Time= np.linspace(0,self.Time_tot,self.N_points) 
        self.N_models= 8
        self.centers=np.linspace(0,self.Time_tot,self.N_models)

        

    def learn_lwr(self): 
        self.cov_mat=0.01
        self.A_k=[]
        self.W_k=[]

        for K in range(self.N_models):
       
            W_k=np.zeros((self.N_points,self.N_points))
        
            for i in range(self.N_points):
                
                phi_k= math.exp( -0.5 * 1/self.cov_mat*  math.pow((self.Time[i]-self.centers[K]),2)   )
                phi_k= phi_k / self.sum_phi(self.Time[i])
                W_k[i,i]= phi_k
            

            self.X = np.column_stack((np.ones(self.N_points), self.Time))
            
            temp= self.X.T @ W_k @ self.X 
            
            A_k =  np.linalg.inv(temp) @ self.X.T @ W_k @ self.Des_traj 
            
          
            self.A_k.append(A_k)
            self.W_k.append(W_k)
        
        
    def regression(self):
        
         sum= np.zeros( [self.N_points, self.Des_traj.shape[1] ] )
         
         for i in range(self.N_models): 

                
                sum= sum + self.W_k[i] @ self.X @ self.A_k[i]
        
     
         file_path = "/home/dhrikarl/Codes/ros_tutorials_ws/src/turtle_control/data/demo_learnt.txt" 
         with open(file_path, "w") as file:
              for row in sum:
                    file.write(" ".join(map(str, row)) + "\n")


              
         return sum

              
         
                

    def sum_phi(self,x) :
            sum=0 
            for k in range(self.N_models):
                    phi_temp= math.exp( -0.5 * 1/self.cov_mat*  math.pow((x-self.centers[k]),2)  )
                    sum  += phi_temp
            return sum
    
    # def command_trajectory(self,Des_traj):
        
    #      self.traj_pub=rospy.Publisher("/turtle1/des_traj_pub",Pose, queue_size=10)  
    #      count = 0 
    #      des_pose_msg=Pose() 
    #      while not rospy.is_shutdown() and count < len(self.Des_traj): 
            
    #         des_pose = self.Des_traj[count,:] 
    #         des_pose_msg.x=des_pose[0] 
    #         des_pose_msg.y=des_pose[1] 
    #         self.traj_pub.publish(des_pose_msg)
    #         count +=1 
    #         print("publishing !!!")
    #         self.rate.sleep() 

              

         



if __name__ == '__main__':

    current_directory = os.getcwd()

    print("Current working directory:", current_directory)
    
    model_file= "/home/dhrikarl/Codes/ros_tutorials_ws/src/turtle_control/data/demo.txt" 
    Des_traj=np.genfromtxt(model_file) /1


    MyDmPLearner = lwr(model_file)
    MyDmPLearner.learn_lwr()
    Learnt_traj= MyDmPLearner.regression()  
    # MyDmPLearner.command_trajectory(Learnt_traj )
    

   
    

    plt.plot(Learnt_traj)
    plt.plot(Des_traj, '--')
    plt.show()
  
  
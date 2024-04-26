#!/usr/bin/env python

import rospy
from franka_LfD.srv import learn_traj, learn_trajResponse
from franka_LfD.lwr import lwr

def learn_traj_func(req):
    model_file= "/home/dhrikarl/Codes/frankhia_ws/src/franka_LfD/data/rob_pose.txt" 

    lwr_learner = lwr(model_file) 
    lwr_learner.learn_lwr()
     



    rospy.loginfo("Learned trajectory !!")
    return learn_trajResponse(1)

def learn_traj_server():
    rospy.init_node('learn_traj_server')
    rospy.loginfo("initalized  !!")
   
    rospy.Service('learn_traj', learn_traj, learn_traj_func)
    rospy.spin()

if __name__ == "__main__":
    model_file= "/home/dhrikarl/Codes/franka_ws/src/franka_LfD/data/rob_pose.txt" 

    lwr_learner = lwr(model_file) 
    lwr_learner.learn_lwr()
    
    learn_traj_server() 



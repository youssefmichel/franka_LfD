#pragma once 

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/package.h>
#include <eigen3/Eigen/Dense>





#include <gripper_controller.h> 


int main(int argc, char *argv[])
{ 
    ros::init(argc, argv,"gripper_controller_node") ;
    ros::NodeHandle nh ;
    string mode ;
    ros::param::get("mode",mode) ;
    string interface_type ;
    ros::param::get("interface", interface_type) ;
  
  if(mode=="tele" && interface_type=="joy"){
    franka_LfD::GripperControllerButton MygripperButton ;
    MygripperButton.init(nh) ;
    MygripperButton.updateGripper() ;
  }
  
  else if(mode== "auto"){
  franka_LfD::GripperControllerTrajectory MygripperController ;
  string pack_path = ros::package::getPath("franka_LfD") ;
  string file_name= pack_path + "/data/gripper_state_demo.txt" ;
  MygripperController.init(file_name) ;
  MygripperController.followReferenceTrajectory() ;
  }
  
  
  return 0 ;


}
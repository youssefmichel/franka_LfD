#pragma once 

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/package.h>
#include <eigen3/Eigen/Dense>





#include "skill_traj_publisher.h" 


int main(int argc, char *argv[])
{ 
  ros::init(argc, argv,"skill_traj_publisher_node") ;
  ros::NodeHandle nh ;
   ros::NodeHandle node_handle("~");
  hardware_interface::RobotHW* robot_hw ;


  // if( ! robot_hw->init(nh, node_handle) ) 
  //   ROS_ERROR("franka_control_node: Failed to initialize FrankaHW class. Shutting down!"); 


  franka_LfD::skill_publisher my_skill_pub  ;
  my_skill_pub.init(2,nh, robot_hw ) ;

  
  sleep(2) ;
  my_skill_pub.publish_des_traj() ;

  return -1 ;


}
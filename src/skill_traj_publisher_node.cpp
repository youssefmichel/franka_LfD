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
  hardware_interface::RobotHW* robot_hw ;


  franka_LfD::SkillPublisherGripper my_skill_pub  ;
  my_skill_pub.init(nh, robot_hw ) ;
  sleep(1) ;
  my_skill_pub.skillRollout() ;
  return -1 ;

}
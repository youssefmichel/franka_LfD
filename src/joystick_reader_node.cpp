#pragma once 

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/package.h>
#include <eigen3/Eigen/Dense>





#include <joystick_reader.h> 


int main(int argc, char *argv[])
{ 
  ros::init(argc, argv,"joystick_reader_node") ;
  ros::NodeHandle nh ;
  franka_LfD::joystick_reader my_joystick_reader = franka_LfD::joystick_reader(3,nh) ; 
  my_joystick_reader.publish_ref_position() ;

  return 0 ;


}
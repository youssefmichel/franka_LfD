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
    // franka_gripper::GraspGoal grasp_goal ;
    // grasp_goal.force =  2.0 ;
    // grasp_goal.width= 0.03 ;

    // grasp_goal.epsilon.inner =0.005;
    // grasp_goal.epsilon.outer = 0.005; 
    // grasp_goal.speed=0.1 ;


    // GraspClient MyGraspClient_("/franka_gripper/grasp", true) ;
    // MyGraspClient_.waitForServer(ros::Duration(3.0)) ;
    // MyGraspClient_.sendGoal(grasp_goal);
    // if (MyGraspClient_.waitForResult(ros::Duration(5.0))) {
    //         return true ;
    //         } 

    //     else {
    //             ROS_WARN("Could Not Grasp Object !!") ;
    //             return false ;
    //         }

    // ros::Publisher ros_pub = nh.advertise<franka_gripper::GraspGoal>("/franka_gripper/grasp/goal",100);
    // ros_pub.publish(grasp_goal) ;

    franka_LfD::GripperControllerBase MyGripperController ;
 // MyGripperController.init() ;
    MyGripperController.gripperTestRun() ;

  
  
  return 0 ;


}

#include <iostream>

#include <ros/ros.h>

 #include "franka_LfD/learn_traj.h"



class test_server {

private: 
ros::ServiceClient client  ;
franka_LfD::learn_traj srv;
ros::NodeHandle nh ;



public: 

void call_service() {

client = nh.serviceClient<franka_LfD::learn_traj>("learn_traj");
franka_LfD::learn_traj srv;
      if (client.call(srv))
    {
        ROS_INFO("Called Trajectory Learner service");
    }
    else
    {
        ROS_ERROR("Failed to call service add_two_ints");
     
    }

}



} ;




int main(int argc, char *argv[])
{ 
  ros::init(argc, argv,"test_serv") ;

sleep(1) ;

  ROS_INFO("-----------------__TEST_----------------------") ;


test_server test ;
test.call_service() ;

}

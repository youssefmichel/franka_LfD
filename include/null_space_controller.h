#pragma once 

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include <franka_LfD/compliance_paramConfig.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <utility.h>

#include <ros/package.h> 

namespace franka_LfD { 

class NullSpaceController  {



public: 


                                               
realtype computeManipIndex( const Mat &J) ;
realtype computeManipIndex ( const franka::RobotState &state  ) ;

Vec findOptimalConfig(std::unique_ptr<franka_hw::FrankaModelHandle> &model_handle, franka::RobotState rob_state)  ;
NullSpaceController( std::unique_ptr<franka_hw::FrankaModelHandle> &model_handle)  ;

private: 

std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
realtype manip_index_prev_ ;
Vec q_prev_ ;





} ;


}
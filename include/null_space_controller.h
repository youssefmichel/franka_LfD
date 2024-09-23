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

/**
 * @class NullSpaceController
 * @brief perfomans singularity optimization
 * 
 * The class is responsible for obtaining the optimal joint space config. 
 * that maximizes the manipubaility index. This is then used in the cartesian 
 * impedance controller to implement the null space action
 */

namespace franka_LfD { 

class NullSpaceController  {

public: 
                                         
    float computeManipIndex( const Mat &J) ;
    Vec findOptimalConfig(std::unique_ptr<franka_hw::FrankaModelHandle> &model_handle, franka::RobotState rob_state)  ;
    NullSpaceController( std::unique_ptr<franka_hw::FrankaModelHandle> &model_handle)  ;
    NullSpaceController() ;


private: 

    enum gradDirection {

        RIGHT=-1, 
        LEFT=1

    } ;

    gradDirection curr_grad_direc_ ;
    float manip_index_prev_ ;
    Vec q_prev_ ;
    franka::RobotState findDirection(std::unique_ptr<franka_hw::FrankaModelHandle> &model_handle,  franka::RobotState rob_state)  ;
    Mat GetJacobianEigen(std::unique_ptr<franka_hw::FrankaModelHandle> &model_handle, franka::RobotState rob_state) ;
    bool Initial_point_optimal_flag_  ; 
    Vec GetKernel(Mat J) ;
    void removeColumn(Eigen::MatrixXd& matrix, unsigned int colToRemove) ;





} ;


}
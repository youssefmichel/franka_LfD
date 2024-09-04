// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
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
#include "franka_LfD/learn_traj.h"
#include  "null_space_controller.h"


namespace franka_LfD {

class CartesianImpedanceController : public controller_interface::MultiInterfaceController<
                                                franka_hw::FrankaModelInterface,
                                                hardware_interface::EffortJointInterface,
                                                franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void stopping(const ros::Time&) override;

 private:
  // Saturation
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;
  std::vector<std::vector<float>> Des_traj_vec_temp_ ; 
  int file_counter_des_ ;

  double filter_params_{0.005};
  double nullspace_stiffness_{20.0};
  double nullspace_stiffness_target_{20.0};
  const double delta_tau_max_{1.0};
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
  Eigen::Matrix<double, 7, 1> q_d_nullspace_;
  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_;
  std::mutex position_and_orientation_d_target_mutex_;
  Eigen::Vector3d position_d_target_;
  Eigen::Quaterniond orientation_d_target_;
  franka_LfD::NullSpaceController null_space_controller_ ;

  ros::Publisher marker_pose_pub_ ;
  visualization_msgs::Marker act_pos_lines_ ;
  

  ros::Subscriber des_pose_sub_ ;


   std::vector <std::vector <float> > pose_vector_ ;
   std::vector <std::vector <float> > pose_quat_vector_ ;
   std::string pose_file_ ;
   std::string pose_file_quat_ ;
   int file_counter_ ;

   void visualize_act_pose(Eigen::Vector3d act_pose) ;

//   ros::ServiceClient client ;
//   franka_LfD::learn_traj srv;


  
  


  


  // Dynamic reconfigure
  std::unique_ptr<dynamic_reconfigure::Server<franka_LfD::compliance_paramConfig>>
      dynamic_server_compliance_param_;
  ros::NodeHandle dynamic_reconfigure_compliance_param_node_;
  void complianceParamCallback(franka_LfD::compliance_paramConfig& config,
                               uint32_t level);
                               

  // Equilibrium pose subscriber
  ros::Subscriber sub_equilibrium_pose_;
  void equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void equilibriumPoseCallback_learned(const geometry_msgs::PoseStampedConstPtr& msg);
};

}  // namespace franka_example_controllers

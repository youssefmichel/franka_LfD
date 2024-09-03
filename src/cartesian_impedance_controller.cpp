// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE



#include <cmath>
#include <memory>


#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <cartesian_impedance_controller.h>

#include <pseudo_inversion.h>
#include <ros/package.h> 


namespace franka_LfD {


bool CartesianImpedanceController::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {
  std::vector<double> cartesian_stiffness_vector;
  std::vector<double> cartesian_damping_vector;
  file_counter_=0 ;
  
  ROS_INFO("----------------------------Starting custom !!------------------------------") ;
  ros::Duration(1.0).sleep();
  
  string mode="tele" ; 
  ros::param::get("/mode", mode) ;
  std::string packPath = ros::package::getPath("franka_LfD");


  // Visualization
  marker_pose_pub_ = node_handle.advertise<visualization_msgs::Marker>("visualization_marker",10) ;
  act_pos_lines_.header.frame_id= "panda_link0"  ;
  act_pos_lines_.ns="franka_act_pose_viz" ;
  act_pos_lines_.scale.x=0.003 ;
  act_pos_lines_.scale.y=0.003 ;
  act_pos_lines_.pose.orientation.w= 1.0 ;
  act_pos_lines_.id=1 ; 
  act_pos_lines_.type= visualization_msgs::Marker::LINE_STRIP;  
  act_pos_lines_.color.r = 1.0;
  act_pos_lines_.color.a = 1.0;



  // Set desired pose
  if(mode== "auto") {
    sub_equilibrium_pose_ = node_handle.subscribe(
      "/franka/des_pose", 20, &CartesianImpedanceController::equilibriumPoseCallback_learned, this,
      ros::TransportHints().reliable().tcpNoDelay()); 

      ROS_INFO("Current Control Mode: Auto") ; 
      pose_file_= packPath + "/data/rob_pose_actual.txt" ;
      pose_file_quat_= packPath + "/data/rob_pose_quat_actual.txt" ;

      } 
  else {

       sub_equilibrium_pose_ = node_handle.subscribe(
       "equilibrium_pose", 20, &CartesianImpedanceController::equilibriumPoseCallback, this,
       ros::TransportHints().reliable().tcpNoDelay());
      

      pose_file_= packPath + "/data/rob_pose_demo.txt" ;
      pose_file_quat_= packPath + "/data/rob_pose_quat_demo.txt" ;

      ROS_INFO("Current Control Mode: Tele") ; 
  }



// definition for FrankaModel
  std::string arm_id;
  //client = node_handle.serviceClient<franka_LfD::learn_traj>("learn_traj");; 
  
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("CartesianImpedanceController: Could not read parameter arm_id");
    return false;
  }
  std::vector<std::string> joint_names;

  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "CartesianImpedanceController: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "CartesianImpedanceController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  dynamic_reconfigure_compliance_param_node_ =
      ros::NodeHandle(node_handle.getNamespace() + "/dynamic_reconfigure_compliance_param_node");

  dynamic_server_compliance_param_ = std::make_unique<
      dynamic_reconfigure::Server<franka_LfD::compliance_paramConfig>>(

      dynamic_reconfigure_compliance_param_node_);
  dynamic_server_compliance_param_->setCallback(
      boost::bind(&CartesianImpedanceController::complianceParamCallback, this, _1, _2));

  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();


  string pose_file= packPath + "/data/skill_0.txt" ;
  general_utility::loadVectorMatrixFromFile(pose_file, 3,  Des_traj_vec_temp_) ;
  file_counter_des_= 0 ;
  


  return true;
}


void CartesianImpedanceController::visualize_act_pose(Eigen::Vector3d act_pose) {

  act_pos_lines_.header.stamp=ros::Time::now() ;
  geometry_msgs::Point point ; 
  point.x=act_pose(0) ;
  point.y=act_pose(1) ;
  point.z=act_pose(2) ;
  act_pos_lines_.action=visualization_msgs::Marker::ADD ;
  act_pos_lines_.points.push_back(point) ;
  marker_pose_pub_.publish(act_pos_lines_) ;


}


void CartesianImpedanceController::starting(const ros::Time& /*time*/) {
  // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
  franka::RobotState initial_state = state_handle_->getRobotState();
  
  // get jacobian
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // set equilibrium point to current state
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.rotation());
  position_d_target_ = initial_transform.translation();
  orientation_d_target_ = Eigen::Quaterniond(initial_transform.rotation());
  //orientation_d_target_.coeffs()<<0.354 ,0.935 ,-0.000, 0.001 ;

  Eigen::Map<Eigen::Matrix<double, 6, 7>> J(jacobian_array.data());
  franka_LfD::NullSpaceController null_space_controller ;
  
  
  // realtype manip = null_space_controller.computeManipIndex( initial_state)  ;

  // cout <<"Manip Index: "<<manip <<endl ;

  
  // const franka_hw::ModelBase* model ; 
  // std::array<double, 42> jacobian_array2 = model->zeroJacobian(franka::Frame::kEndEffector, initial_state ) ;

  // Eigen::Map<Eigen::Matrix<double, 6, 7>> J2(jacobian_array2.data()); 

  //  cout<<"Manip Index: " << J2<<endl ; 



  // set nullspace equilibrium configuration to initial q
  q_d_nullspace_ = q_initial;



}

void CartesianImpedanceController::update(const ros::Time& /*time*/,
                                                 const ros::Duration& /*period*/) {
  // get state variables


  franka::RobotState robot_state = state_handle_->getRobotState();
  int length = Des_traj_vec_temp_.size() ;
  string mode="tele" ; 
  ros::param::get("/mode", mode) ;

if(mode=="auto" ){
  if(file_counter_des_<length-1) {
    
    // position_d_target_[0] = Des_traj_vec_temp_[file_counter_des_][0]  ;
    // position_d_target_[1] = Des_traj_vec_temp_[file_counter_des_][1]  ;
    // position_d_target_[2] = Des_traj_vec_temp_[file_counter_des_][2]  ;
    // file_counter_des_ ++ ;

  }
  }


  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.rotation());

  // compute error to desired pose
  // position error
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << position - position_d_;

  // orientation error
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // Transform to base frame
  error.tail(3) << -transform.rotation() * error.tail(3);

  // compute control
  // allocate variables
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);

  // pseudoinverse for nullspace handling
  // kinematic pseuoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  // Cartesian PD control with damping ratio = 1
  tau_task << jacobian.transpose() *
                  (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
  // nullspace PD control with damping ratio = 1
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       (nullspace_stiffness_ * (q_d_nullspace_ - q) -
                        (2.0 * sqrt(nullspace_stiffness_)) * dq);

  // Desired torque
  tau_d << tau_task + tau_nullspace + coriolis;

  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }

  pose_vector_.push_back(std::vector<float>()) ;
  pose_vector_[file_counter_].push_back(position[0]) ;
  pose_vector_[file_counter_].push_back(position[1]) ;
  pose_vector_[file_counter_].push_back(position[2]) ;
  pose_vector_[file_counter_].push_back(position_d_[0]) ;
  pose_vector_[file_counter_].push_back(position_d_[1]) ;
  pose_vector_[file_counter_].push_back(position_d_[2]) ;

  pose_quat_vector_.push_back(std::vector<float>()) ;
  pose_quat_vector_[file_counter_].push_back(orientation.w()) ;
  pose_quat_vector_[file_counter_].push_back(orientation.x()) ;
  pose_quat_vector_[file_counter_].push_back(orientation.y()) ;
  pose_quat_vector_[file_counter_].push_back(orientation.z()) ;

  Vec velocity= jacobian * dq ;
  pose_quat_vector_[file_counter_].push_back(velocity[3]) ;
  pose_quat_vector_[file_counter_].push_back(velocity[4]) ;
  pose_quat_vector_[file_counter_].push_back(velocity[5]) ;
  file_counter_++ ;


  // update parameters changed online either through dynamic reconfigure or through the interactive
  // target by filtering
  cartesian_stiffness_ =
      filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
  cartesian_damping_ =
      filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
  nullspace_stiffness_ =
      filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;

   cartesian_stiffness_ =cartesian_stiffness_target_ ;
   cartesian_damping_ = cartesian_damping_target_ ;
   nullspace_stiffness_ = nullspace_stiffness_target_ ;

  std::lock_guard<std::mutex> position_d_target_mutex_lock(
      position_and_orientation_d_target_mutex_);

 position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
 position_d_=position_d_target_ ;

 orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
 //orientation_d_=orientation_d_target_ ;
//  cout<<"Desired Orientation: " <<orientation_d_.coeffs().transpose()<<endl ;
//  cout<<"Act Orientation: " <<orientation.coeffs().transpose()<<endl ;



  //visualize_act_pose(position) ;
  


}

void CartesianImpedanceController::stopping(const ros::Time& /*time*/) { 

      ROS_INFO("-------------------Saving file -------------------!! ") ;
      general_utility::saveVectorMatrixToFile(pose_file_,pose_vector_) ;
      general_utility::saveVectorMatrixToFile(pose_file_quat_,pose_quat_vector_) ;

    //  if (client.call(srv))
    //  {
    //      ROS_INFO("Called Trajectory Learner service");
    //  }
    // else
    //  {
    //      ROS_ERROR("Failed to call service Traj. Learner");

    // }

                                              
 }



Eigen::Matrix<double, 7, 1> CartesianImpedanceController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

void CartesianImpedanceController::complianceParamCallback(
    franka_LfD::compliance_paramConfig& config,
    uint32_t /*level*/) {
  cartesian_stiffness_target_.setIdentity();
  cartesian_stiffness_target_.topLeftCorner(3, 3)
      << 2000 * Eigen::Matrix3d::Identity();
      
  cartesian_stiffness_target_.bottomRightCorner(3, 3)
      << 150 * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.setIdentity();
 
  cartesian_damping_target_.topLeftCorner(3, 3)
      << 2.0 * sqrt(config.translational_stiffness) * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.bottomRightCorner(3, 3)
      << 2.0 * sqrt(config.rotational_stiffness) * Eigen::Matrix3d::Identity();
  nullspace_stiffness_target_ = config.nullspace_stiffness;
 
}

void CartesianImpedanceController::equilibriumPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  std::lock_guard<std::mutex> position_d_target_mutex_lock(
      position_and_orientation_d_target_mutex_);
  position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);

  orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;

   if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
     orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
   }

  

}

void CartesianImpedanceController::equilibriumPoseCallback_learned(
    const geometry_msgs::PoseStampedConstPtr& msg) {

  std::lock_guard<std::mutex> position_d_target_mutex_lock(
      position_and_orientation_d_target_mutex_);
 
  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

//  orientation_d_target_.coeffs()<< msg->pose.orientation.w, msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z ; 
  
//     if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
//      orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
//    }

//     orientation_d_target_.coeffs() <<0,0,0,1 ;

}


}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_LfD::CartesianImpedanceController,
                       controller_interface::ControllerBase)

#pragma once 

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/package.h>
#include <eigen3/Eigen/Dense>
#include <franka/robot_state.h>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>


#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <visualization_msgs/Marker.h>
#include <gripper_controller.h>

#include <utility.h> 

/**
 * @class SkillPublisher 
 * @brief publishes the learnt skill to the robot
 * 
 * The base class is responsible for simply
 * orientations for the robot
 */


namespace franka_LfD { 

    class SkillPublisher{

        protected: 
            ros::Publisher des_traj_pub_ ;
            int n_skills_ ;
            std::vector <std::vector <std::vector <float>>> Des_traj_list_ ;
            std::vector <std::vector <std::vector <float>>> Des_traj_quat_list_ ;
            std::vector<std::vector<float>> test_des_ ;
            geometry_msgs::PoseStamped des_pose_msg_ ;
            
            void update_act_rob_pose_callback(geometry_msgs::PoseStamped::ConstPtr& act_rob_pose) ;
            ros::Subscriber act_rob_pose_sub ;
            std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
            Eigen::Vector3d current_robot_position_ ;
            void get_current_robot_state(Eigen::Vector3d& position  ) ;
            void visualize_des_pose() ;
            visualization_msgs::Marker points_ ;
            ros::Publisher pose_visualizer_pub_ ;
            bool readSkillsfromFile(string packPath) ;
            void publish_traj_segment(const std::vector <std::vector <float>>& curr_traj, const std::vector <std::vector <float>>& curr_traj_quat) ;
           
    
        public: 

            bool  init (int n_skills, ros::NodeHandle nh, hardware_interface::RobotHW* robot_hw ) ;
             ~SkillPublisher() ;
             int publish_des_traj() ; 


    } ;

    class SkillPublisherGripper: public SkillPublisher{
        
        private: 
        
            int N_movements_ ;
            int N_gripper_actions_; 
            int Nstates_ ;
            std::vector<char> state_list_ ; 
            GripperControllerBase MyGripperController_ ;
        
        public: 
        
            bool  init (ros::NodeHandle nh, hardware_interface::RobotHW* robot_hw ) ;
            bool skillRollout() ;
          
            

    } ;

}

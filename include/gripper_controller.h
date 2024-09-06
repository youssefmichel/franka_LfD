/**
 * Control implementation for gripper 
 * Provides Base class implementation
 * Gripper can be either controller via joystick, or from a reference trajectory
 */

#include <franka/gripper.h>
#include <franka_example_controllers/teleop_gripper_paramConfig.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/HomingAction.h>
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/StopAction.h>

#include <actionlib/client/simple_action_client.h>
#include <dynamic_reconfigure/server.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <sensor_msgs/JointState.h>

#include <functional>
#include <memory>
#include <mutex>
#include <utility.h>
#include <ros/package.h>
#include <sensor_msgs/Joy.h>

using franka_gripper::GraspAction;
using franka_gripper::HomingAction;
using franka_gripper::MoveAction;
using franka_gripper::StopAction;

using GraspClient = actionlib::SimpleActionClient<GraspAction>;
using HomingClient = actionlib::SimpleActionClient<HomingAction>;
using MoveClient = actionlib::SimpleActionClient<MoveAction>;
using StopClient = actionlib::SimpleActionClient<StopAction>;
# define MAX_GRIPPER_WIDTH 0.07 
# define MIN_GRIPPER_WIDTH 0.03


namespace franka_LfD{

    class GripperControllerBase {

        protected: 
            
            bool SubscriberCallback(const sensor_msgs::JointState& msg) ;  
            
            bool homingGripper() ;
            bool GraspObject() ;
            bool GripperAction(realtype ref_width, realtype ref_force) ; 
            bool ReleaseObject() ;
            bool StopGrasp() ;
            double commanded_width_ ;
            double commanded_force_ ;
            
            

        public: 
      
            bool init() ;
            void gripperTestRun() ;


    } ;

    class GripperControllerButton:public GripperControllerBase{

        private:
            ros::Subscriber joystick_sub_ ; 
            void joyButtonCallback(const sensor_msgs::Joy::ConstPtr& joy); 
            
            bool flag_close_gripper_  ;
            bool flag_open_gripper_ ;

        public: 

            void updateGripper() ;
            bool init(ros::NodeHandle nh) ;

    } ;

    class GripperControllerTrajectory:public GripperControllerBase {

        private:
            std::vector<std::vector<float> > ref_trajectory_ ;
        public :
            bool init(std::string trajectory_file) ;
            void followReferenceTrajectory() ;
    } ;

    
}
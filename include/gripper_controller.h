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
# define MIN_GRIPPER_FORCE 0.0 
# define MAX_GRIPPER_FORCE 2.0

/**
 * @class GripperController
 * @brief implements gripper-related functionality 
 * 
 * The base class provides basic gripper primitives, such as homing, grasping and release of objects
 * The child class GripperControllerButton controllers the gripper based on button
 * The other child class controls the gripper to follow a reference trajectory 
 * consisting of reference width and forces
 */


namespace franka_LfD{

    class GripperControllerBase {

        protected: 
            
            bool SubscriberCallback(const sensor_msgs::JointState& msg) ;  
            
            bool homingGripper() ;
            bool GraspObject() ;
            
            bool ReleaseObject() ;
            bool StopGrasp() ;
            double commanded_width_ ;
            double commanded_force_ ;
           

        public: 
      
            bool init() ;
            void gripperTestRun() ;
            bool GripperAction(realtype ref_width, realtype ref_force) ; 


    } ;

    class GripperControllerButton:public GripperControllerBase{

        private:
            ros::Subscriber joystick_sub_ ; 
            void joyButtonCallback(const sensor_msgs::Joy::ConstPtr& joy); 
            
            bool flag_close_gripper_  ;
            bool flag_open_gripper_ ;
            std::vector<std::vector<float> > gripper_state_vector_ ;

        public: 

            void updateGripper() ;
            bool init(ros::NodeHandle nh) ;

    } ;

    class GripperControllerTrajectory:public GripperControllerBase {

        private:
            std::vector<std::vector<float> > ref_trajectory_ ;
            std::vector<std::vector<float> > act_trajectory_ ;
        public :
            bool init(std::string trajectory_file) ;
            void followReferenceTrajectory() ;
    } ;

    
}
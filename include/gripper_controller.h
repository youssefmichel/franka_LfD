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


using franka_gripper::GraspAction;
using franka_gripper::HomingAction;
using franka_gripper::MoveAction;
using franka_gripper::StopAction;
using GraspClient = actionlib::SimpleActionClient<GraspAction>;
using HomingClient = actionlib::SimpleActionClient<HomingAction>;
using MoveClient = actionlib::SimpleActionClient<MoveAction>;
using StopClient = actionlib::SimpleActionClient<StopAction>;

namespace franka_LfD{

    class GripperControllerBase {

        protected: 
            GraspClient MyGraspClient_ ;
            bool SubscriberCallback(const sensor_msgs::JointState& msg) ;  


        public: 
      
            bool init(const std::shared_ptr<ros::NodeHandle>& pnh) ;

        

    } ;
}
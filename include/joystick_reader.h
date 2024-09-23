#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <utility.h> 
#include <geometry_msgs/PoseStamped.h>

#include <franka_msgs/FrankaState.h>

/**
 * @class joystick_reader 
 * @brief reads and interprets data from joystick 
 * 
 * The class is responsible for reading data from joystick, computing reference positions and
 * orientations and pubishing them for the robot 
 */

namespace franka_LfD {
class joystick_reader {

public: 
    void publish_ref_position() ;
    joystick_reader(int dim, ros::NodeHandle nh ) ;
    ros::NodeHandle nh ; 
    
private: 
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy); 
    void initialStateCallback(const franka_msgs::FrankaState::ConstPtr& franka_state) ; 

    Vec  omega2quat(Vec quat, Vec q) ;
    ros::Subscriber joystick_sub_ ;
    ros::Subscriber initial_state_sub_ ;
    bool flag_state_initalized_ ;

    ros::Publisher ref_pos_pub_ ; 
    realtype resolveXaxis(double axis_1, double axis_2) ;
    Vec joystick_inp_ ;
    Vec joystick_inp_rot_ ; // rotational input (angluar vel)
    Vec rob_pos_init_ ;
    Vec rob_pos_des_  ;
    Vec rob_quat_des_ ;
    Vec rob_quat_init_ ;
    bool flags_x_inp_[2] ; // needed to handle joy stick inputs for the x direction

    
    
    


} ;
}
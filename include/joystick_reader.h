#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <utility.h> 
#include <geometry_msgs/PoseStamped.h>

namespace franka_LfD {
class joystick_reader {

public: 
    void publish_ref_position() ;
    joystick_reader(int dim, ros::NodeHandle nh ) ;
    ros::NodeHandle nh ;

    
    

    

private: 
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy); 
    ros::Subscriber joystick_sub_ ;
    ros::Publisher ref_pos_pub_ ; 
    
    
    
    Vec joystick_inp_ ;
    Vec rob_pos_init_ ;
    
    
    


} ;
}
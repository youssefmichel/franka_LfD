#include "joystick_reader.h" 

namespace franka_LfD {

    joystick_reader::joystick_reader(int dim, ros::NodeHandle nh) {
            
        joystick_inp_=Vec::Zero(dim) ;
        rob_pos_init_= Vec::Zero(dim) ; // @TODO: set to initial robot position
        rob_pos_init_<<0.307, -0.015 ,0.488 ;
        
        joystick_sub_ =  nh.subscribe("/joy", 20,
         &joystick_reader::joyCallback, this, ros::TransportHints().reliable().tcpNoDelay()); 

         ref_pos_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/franka/des_pose",10 ) ; 
         

        ROS_INFO("Joystick Reader Node initialized !") ;
    }

    void joystick_reader::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {

        // We interpret the joystick input as a ref velocity 
        joystick_inp_(0) = joy->axes[7] ;
        joystick_inp_(1) = joy->axes[3] ;
        joystick_inp_(2) = joy->axes[4] ;

    }

    void joystick_reader::publish_ref_position() {
        
        geometry_msgs::PoseStamped des_pose_msg ;
        ros::Rate loop_rate(1000);  
        Mat scale=Mat::Identity(3,3) ; 
        scale(0,0)=10 ;
        scale(1,1)=-10 ;
        scale(2,2)=10 ;
        

        while(ros::ok()) {
            realtype dt=0.01 ;
            Vec ref_pos  = rob_pos_init_ + dt*scale*joystick_inp_ ; // we treate joystick input as a ref. vel 
            des_pose_msg.pose.position.x= ref_pos(0) ;
            des_pose_msg.pose.position.y= ref_pos(1) ;
            des_pose_msg.pose.position.z= ref_pos(2) ;
            ref_pos_pub_.publish(des_pose_msg) ;
            loop_rate.sleep() ;
            ros::spinOnce() ;
           

        }

    }


   
}
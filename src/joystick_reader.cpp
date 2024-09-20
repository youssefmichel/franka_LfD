#include "joystick_reader.h" 

namespace franka_LfD {

    joystick_reader::joystick_reader(int dim, ros::NodeHandle nh) {
            
        joystick_inp_=Vec::Zero(dim) ;
        joystick_inp_rot_ = Vec::Zero(4) ;
        flags_x_inp_[0] =false ;
        flags_x_inp_[1] =false ;
        flag_state_initalized_ =false ;

        joystick_sub_ =  nh.subscribe("/joy", 20,
         &joystick_reader::joyCallback, this, ros::TransportHints().reliable().tcpNoDelay()); 

        initial_state_sub_ =nh.subscribe("/franka_state_controller/franka_states",20,
         &joystick_reader::initialStateCallback, this, ros::TransportHints().reliable().tcpNoDelay()) ;

        ref_pos_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/franka/des_pose",10 ) ; 
        rob_pos_init_= Vec::Zero(dim) ; 
        rob_quat_init_ =Vec::Zero(4) ;
        bool flag_des_init=false ;

         //should be set by callback
        while( flag_state_initalized_ ==false || flag_des_init==false)  {
        
        if(flag_state_initalized_) {    
        rob_pos_des_ = rob_pos_init_ ;
        rob_quat_des_ =rob_quat_init_ ;
        flag_des_init=true ;
        }

        ros::spinOnce() ;
        
        }  


         

        ROS_INFO("Joystick Reader Node initialized !") ;
    }

    void joystick_reader::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {

        // We interpret the joystick input as a ref velocity 

        joystick_inp_(0) = resolveXaxis(joy->axes[2], joy->axes[5]);
        joystick_inp_(1) = joy->axes[3] ;
        joystick_inp_(2) = joy->axes[4] ;

        joystick_inp_rot_(1) = 0.0 ;
        joystick_inp_rot_(0) = joy->axes[0] ;
        joystick_inp_rot_(2) = joy->axes[1] ;

    }

    realtype joystick_reader::resolveXaxis(double axis_1, double axis_2) {

        // intereprets the joystick input in the x direction. 
        // Logic is a bit elaborate, but due to the rather weird behavior of RT/LT joystick inputs

        if (flags_x_inp_[0] == false || flags_x_inp_[1] == false   ) {

            if(abs(axis_1)>0.00001) {
                flags_x_inp_[0]=true ;
            }

             if(abs(axis_2)>0.00001) {
                flags_x_inp_[1]=true ;
            }
        }

         if (flags_x_inp_[0] == true ) {
            realtype diff = abs(1 -axis_1) ;

            if( diff > 0.0001 ) {
                    return diff ; 
            }
         }

        if (flags_x_inp_[1] == true ) {
            realtype diff = abs(1 -axis_2) ;

            if( diff > 0.0001 ) {
                    return -diff ; 
            }
         }

         return 0 ;


    }

    void joystick_reader::initialStateCallback(const franka_msgs::FrankaState::ConstPtr& franka_state) {
        
        if(flag_state_initalized_ == false) {
       Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(franka_state->O_T_EE.data())) ;
       rob_pos_init_ = initial_transform.translation();
       Eigen::Quaterniond quat=  Eigen::Quaterniond(initial_transform.rotation());
       rob_quat_init_<<   quat.coeffs().w(), quat.coeffs().x() , quat.coeffs().y() , quat.coeffs().z() ;
       flag_state_initalized_ =true ;
       }
    }

    void joystick_reader::publish_ref_position() {
        
        geometry_msgs::PoseStamped des_pose_msg ;
        ros::Rate loop_rate(1000);  
        Mat scale=Mat::Identity(3,3) ; 
        scale(0,0)=0.01 ;
        scale(1,1)=-0.01;
        scale(2,2)=0.01;
        
        while(ros::ok()) {
            realtype dt=0.01 ;

        //  Vec ref_pos  = rob_pos_init_ + dt*scale*joystick_inp_ ; // we treate joystick input as a ref. vel 
        //  Vec quat_des = omega2quat(joystick_inp_rot_,rob_quat_init_) ;
         
            rob_quat_des_ = omega2quat(joystick_inp_rot_,rob_quat_des_) ;
            Vec quat_des =rob_quat_des_  ;
    
            rob_pos_des_= rob_pos_des_ + dt*scale * joystick_inp_ ;
            Vec ref_pos= rob_pos_des_ ; 

            des_pose_msg.pose.position.x= ref_pos(0) ;
            des_pose_msg.pose.position.y= ref_pos(1) ;
            des_pose_msg.pose.position.z= ref_pos(2) ;

            des_pose_msg.pose.orientation.w= quat_des(0) ;
            des_pose_msg.pose.orientation.x= quat_des(1) ;
            des_pose_msg.pose.orientation.y= quat_des(2) ;
            des_pose_msg.pose.orientation.z= quat_des(3) ;

            ref_pos_pub_.publish(des_pose_msg) ;
            loop_rate.sleep() ;
            ros::spinOnce() ;
            
        
        }

    }

    Vec joystick_reader::omega2quat(Vec omega, Vec q) {

        Vec eta= 0.5* omega * 0.0006 ;
        double eta_arr[] = {eta(0),eta(1),eta(2) } ;
        double q_arr[] =  {q[0], q[1] ,q[2] , q[3] } ;
        double q_temp[4]  ;
        double q_temp2[4] ;

        quat_utils::quat_exp(eta_arr,q_temp) ;
    
        quat_utils::quat_mult(q_temp, q_arr,q_temp2) ;
        
    
        Vec q_out = Vec::Zero(4) ;
        q_out(0) = q_temp2[0] ;
        q_out(1) = q_temp2[1] ;
        q_out(2) = q_temp2[2] ;
        q_out(3) = q_temp2[3] ;

        return q_out ;


    }


   
}
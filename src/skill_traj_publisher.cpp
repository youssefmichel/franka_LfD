#include "skill_traj_publisher.h" 

namespace franka_LfD { 

    
    
    void skill_publisher::init(int n_skills,ros::NodeHandle nh, hardware_interface::RobotHW* robot_hw) {

        ROS_INFO("Skill Publisher Node !!") ;
        n_skills_=n_skills ;
        string packPath = ros::package::getPath("franka_LfD"); 
        int n_DOF =3  ;
        des_traj_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/franka/des_pose",10) ;  
        pose_visualizer_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_marker",10) ;
        string link_name ;
        if(! ros::param::get("/link_name",link_name) ) {
            ROS_ERROR("Could not set marker frame") ;

        }

        points_.header.frame_id ="panda_link0" ;
        //points_.header.frame_id =link_name ;
        std::string arm_id;
     // client = node_handle.serviceClient<franka_LfD::learn_traj>("learn_traj");; 
  
        if (!nh.getParam("arm_id", arm_id)) {
        ROS_ERROR_STREAM("CartesianImpedanceController: Could not read parameter arm_id");
        }
        

        // const franka_hw::ModelBase* model_;

        // franka::RobotState state ; 
        // model_->zeroJacobian(franka::Frame::kEndEffector,state.q, state.F_T_EE , state.EE_T_K  ) ;
    

        // cout <<"Initial Robot state 1"<<endl ;
        // auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
        
        // state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        // state_interface->getHandle(arm_id + "_robot")); 

        // get_current_robot_state(current_robot_position_) ;
        // cout <<"Initial Robot state" <<current_robot_position_.transpose() <<endl ;


        for (int i=0 ; i <n_skills ; i++ ) {

            string pose_file= packPath + "/data/skill_" + std::to_string(i) + ".txt" ;
            string pose_file_quat= packPath + "/data/skill_quat_" + std::to_string(i) + ".txt" ;
            std::vector<std::vector<float>> temp_vec ;
            general_utility::loadVectorMatrixFromFile(pose_file, n_DOF,  temp_vec) ;
            std::vector<std::vector<float>> temp_vec_quat ;
            general_utility::loadVectorMatrixFromFile(pose_file_quat, 4,  temp_vec_quat) ;
            Des_traj_list_.push_back(temp_vec) ;
            Des_traj_quat_list_.push_back(temp_vec_quat) ;

        }

        sleep(0.1) ;


    }

     void skill_publisher::visualize_des_pose() {
        points_.header.stamp=ros::Time::now() ;
        points_.ns="franka_des_pose_viz" ;
        points_.action=visualization_msgs::Marker::ADD ;
        // points_.pose.position.x= des_pose_msg_.pose.position.x ; 
        // points_.pose.position.y= des_pose_msg_.pose.position.y ; 
        // points_.pose.position.z= des_pose_msg_.pose.position.z ; 
        geometry_msgs::Point curr_point ;
        curr_point.x=des_pose_msg_.pose.position.x ;  
        curr_point.y=des_pose_msg_.pose.position.y ;  
        curr_point.z=des_pose_msg_.pose.position.z ;  

        points_.points.push_back(curr_point) ;
        points_.id= 0 ; 
        points_.type=visualization_msgs::Marker::SPHERE_LIST;
        points_.scale.x=0.005 ;
        points_.scale.y=0.005 ;
        points_.color.g=1.0f ;
         points_.color.a = 1.0;
        points_.pose.orientation.w = 1.0;
        
        pose_visualizer_pub_ .publish(points_);


     }

    skill_publisher::~skill_publisher() {
       ROS_INFO("skill publisher destructed !!") ;
    }

    void skill_publisher::update_act_rob_pose_callback(geometry_msgs::PoseStampedConstPtr& act_rob_pose_msg ) {


    }

    void skill_publisher::get_current_robot_state (Eigen::Vector3d& position ) {
    franka::RobotState robot_state = state_handle_->getRobotState();

    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    position = transform.translation() ;  
 
    }

    int skill_publisher::publish_des_traj() {


         ros::Rate loop_rate(1000);  
        for (int i=0 ; i <n_skills_ ; i++  ) {

            int file_counter= 0 ; 
            std::vector <std::vector <float>> curr_traj = Des_traj_list_.at(i) ; 
            std::vector <std::vector <float>> curr_traj_quat = Des_traj_quat_list_.at(i) ; 
            int dim_traj= curr_traj.size() ;
            realtype t_elap= 0 ;
            
            while(ros::ok() && file_counter < dim_traj-1) {

    
                des_pose_msg_.pose.position.x=curr_traj[file_counter][0]  ;
                des_pose_msg_.pose.position.y=curr_traj[file_counter][1]  ;
                des_pose_msg_.pose.position.z=curr_traj[file_counter][2]  ;
                
                des_pose_msg_.pose.orientation.w=curr_traj_quat[file_counter][0]  ;
                des_pose_msg_.pose.orientation.x=curr_traj_quat[file_counter][1]  ;
                des_pose_msg_.pose.orientation.y=curr_traj_quat[file_counter][2]  ;
                des_pose_msg_.pose.orientation.z=curr_traj_quat[file_counter][3]  ;

                //std::cout<<"Publishing: "<< des_pose_msg_.pose.position.x << " y: "<< des_pose_msg_.pose.position.y <<endl ;
                des_traj_pub_.publish(des_pose_msg_) ;
                if(t_elap>1) {
                visualize_des_pose() ;
                t_elap=0 ;
                }
              
            
                file_counter ++ ;
                loop_rate.sleep() ; 
               
                ros::spinOnce() ;
                t_elap +=0.01 ;
             

            }
            ROS_INFO("finished one !") ;

            }
        ROS_INFO("Published ALL") ;
        return 1 ;




    }


}

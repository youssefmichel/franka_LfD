#include "skill_traj_publisher.h" 

namespace franka_LfD { 

    
    
   bool skill_publisher::init(int n_skills,ros::NodeHandle nh, hardware_interface::RobotHW* robot_hw) {

        ROS_INFO("Skill Publisher Node !!") ;
        n_skills_=n_skills ;
        string packPath = ros::package::getPath("franka_LfD"); 
        des_traj_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/franka/des_pose",10) ;  
        pose_visualizer_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_marker",10) ;
        string link_name ;
        if(! ros::param::get("/link_name",link_name) ) {
            ROS_ERROR("Could not set marker frame") ;

        }

        points_.header.frame_id ="panda_link0" ;
        std::string arm_id;
     // client = node_handle.serviceClient<franka_LfD::learn_traj>("learn_traj");; 
  
        if (!nh.getParam("arm_id", arm_id)) {
        ROS_ERROR_STREAM("CartesianImpedanceController: Could not read parameter arm_id");
        return false ;
        }
        
        if(readSkillsfromFile(packPath)==false) {
            return false ;
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

   

        sleep(0.1) ;
        return true ;


    }

    bool skill_publisher::readSkillsfromFile(string packPath) {
            
            for (int i=0 ; i <n_skills_ ; i++ ) {

            string pose_file= packPath + "/data/skill_" + std::to_string(i) + ".txt" ;
            string pose_file_quat= packPath + "/data/skill_quat_" + std::to_string(i) + ".txt" ;
            std::vector<std::vector<float>> temp_vec ;
           
            if(general_utility::loadVectorMatrixFromFile(pose_file, 3,  temp_vec) == -1){
               ROS_ERROR_STREAM("Trajectory Pose File Not found ! "); 
               return false ;
            } 
            std::vector<std::vector<float>> temp_vec_quat ;

            if(general_utility::loadVectorMatrixFromFile(pose_file_quat, 4,  temp_vec_quat)==-1){
                
                ROS_ERROR_STREAM("Orientation Pose File Not found ! "); 
                return false ;
            } 

            Des_traj_list_.push_back(temp_vec) ;
            Des_traj_quat_list_.push_back(temp_vec_quat) ;

        }
        return true ;
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
         
        for (int i=0 ; i <n_skills_ ; i++  ) {

        
            std::vector <std::vector <float>> curr_traj = Des_traj_list_.at(i) ; 
            std::vector <std::vector <float>> curr_traj_quat = Des_traj_quat_list_.at(i) ; 
            publish_traj_segment(curr_traj,curr_traj_quat  ) ;
        }

        return 1 ;

    }

    bool skill_publisher::publish_traj_segment(const std::vector <std::vector <float>>& curr_traj, const std::vector <std::vector <float>>& curr_traj_quat) {

            int dim_traj= curr_traj.size() ;
            realtype t_elap= 0 ;
            int file_counter= 0 ;
            ros::Rate loop_rate(1000);  
            
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

            }


    



    bool skill_publisher_gripper::init(ros::NodeHandle nh, hardware_interface::RobotHW* robot_hw ) {
           
            string packPath = ros::package::getPath("franka_LfD"); 
            

            std::ifstream state_file(packPath+ "/data/state_list.txt" ) ;

            if (! state_file.is_open()) {
               std::cerr << "Error opening char list file." << std::endl;
                return false ;
             }
              
            char c;
            while (state_file.get(c)) {
            state_list_.push_back(c);
            }

            N_movements_= 0 ;
            Nstates_= state_list_.size() ;
            for(int i=0 ; i<Nstates_ ; i++) {
                    if(state_list_[i] == 'm' ) {
                        N_movements_ ++  ;
                    }
            }

            N_gripper_actions_ = Nstates_  - N_movements_ ;
            if(skill_publisher::init(N_movements_ , nh, robot_hw)==false) {
                return false ;
            }


    }

    bool skill_publisher_gripper::skillRollout() {

        int mov_counter =  0; 
 
        for (int i=0 ; i<Nstates_ ; i++ ) {

            char state=state_list_[i] ; 
                
            std::vector <std::vector <float>> curr_traj = Des_traj_list_.at(mov_counter ) ; 
            std::vector <std::vector <float>> curr_traj_quat = Des_traj_quat_list_.at(mov_counter ) ;
            
            switch (state)
            {
            case 'm':
                
                mov_counter++ ; 
                publish_traj_segment(curr_traj,curr_traj_quat  ) ;
                break;

            case 'o':
               
               break;
            
            case 'c': 
                

            default:
                break;
            }

        }


    }




}

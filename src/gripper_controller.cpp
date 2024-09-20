#include <gripper_controller.h> 


namespace franka_LfD{ 

  

    
    bool GripperControllerBase::init() 
 
    {
    
    ROS_INFO("Gripper Node Launched !!") ;
    commanded_width_= MIN_GRIPPER_WIDTH ;
    commanded_force_ = MIN_GRIPPER_FORCE ;

    return true ;

    }

    void GripperControllerBase::gripperTestRun() {

        init() ;
        
        GraspObject() ;
       // StopGrasp() ;
        ReleaseObject() ;

    }

    bool GripperControllerBase::homingGripper() {

            HomingClient myHomingClient("/franka_gripper/homing", true) ;
            ros::Duration timeout(5.0) ;
            if(myHomingClient.waitForServer(timeout)) {
                myHomingClient.sendGoal(franka_gripper::HomingGoal()) ;
            }

            if(myHomingClient.waitForResult(ros::Duration(10.0))) {
                return true ;
            }

            ROS_ERROR("Could not send gripper home") ;
            return false ;

    }

    bool GripperControllerBase::GraspObject() {

        if(commanded_width_>MIN_GRIPPER_WIDTH) { // should only close if gripper is open 
                commanded_width_ = MIN_GRIPPER_WIDTH ;
                commanded_force_=  MAX_GRIPPER_FORCE ;
                GripperAction(commanded_width_ , commanded_force_) ;
                return true ;
        }
        else {

            return false; 

        }
    }

    bool GripperControllerBase::GripperAction(realtype ref_width, realtype ref_force) {


           franka_gripper::GraspGoal grasp_goal ;
           grasp_goal.force =  ref_force;
     
           
           grasp_goal.width= ref_width ;
           grasp_goal.epsilon.inner =0.005;
           grasp_goal.epsilon.outer = 0.005; 
           grasp_goal.speed=0.05 ;
         
           GraspClient MyGraspClient("/franka_gripper/grasp", true) ;
           MyGraspClient.waitForServer(ros::Duration(2.0)) ;
           MyGraspClient.sendGoal(grasp_goal);
           
           
           if (MyGraspClient.waitForResult(ros::Duration(5.0))) {
                 return true ;
             } 

            else {
                ROS_WARN("Could Not Apply Gripper Action !!") ;
                return false ;
             }

             }

        

     bool GripperControllerBase::ReleaseObject()  {

            if(commanded_width_<MAX_GRIPPER_WIDTH){
                commanded_width_ = MAX_GRIPPER_WIDTH ;
                commanded_force_= MIN_GRIPPER_WIDTH ;
                GripperAction(commanded_width_ , commanded_force_) ;
                return true ;
        
             }
             else{
                return false ;
             }
}
    

    bool GripperControllerBase::StopGrasp() {

        StopClient MyGripperStopper("/franka_gripper/stop",true) ;
        if(MyGripperStopper.waitForServer(ros::Duration(2))) {
            MyGripperStopper.sendGoal(franka_gripper::StopGoal()) ;

            if(MyGripperStopper.waitForResult(ros::Duration(5.0))) {
                ROS_INFO("Stopped Gripper") ;
                return true ;
            } 

        }

        return false; 

    }

    bool GripperControllerButton::init( ros::NodeHandle nh) {

                
                GripperControllerBase::init() ;
               joystick_sub_ =  nh.subscribe("/joy", 20,
                &GripperControllerButton::joyButtonCallback, this, ros::TransportHints().reliable().tcpNoDelay()); 
                flag_close_gripper_  = false ;
                flag_open_gripper_   = false ;
                ROS_INFO("Gripper Node Button !!") ;
                return true ;
    }

    void GripperControllerButton::joyButtonCallback(const sensor_msgs::Joy::ConstPtr& joy) {

   
        if(joy->buttons[2]==1) {
            flag_open_gripper_= true ;
        } 
         
         if(joy->buttons[1]==1) {
            flag_close_gripper_= true ;
        } 

    }

    void GripperControllerButton::updateGripper() {

        ros::Rate loop_rate(1000) ;
        int file_counter(0) ;

        while (ros::ok()) {

                if (flag_close_gripper_) {

                 GraspObject() ;
                 flag_close_gripper_ = false ;

                }
                if(flag_open_gripper_) {

                    ReleaseObject() ;
                    flag_open_gripper_  = false ;
                }

            loop_rate.sleep() ;
            gripper_state_vector_.push_back(std::vector<float>()) ;
            gripper_state_vector_[file_counter].push_back(commanded_width_) ;
            gripper_state_vector_[file_counter].push_back(commanded_force_) ;
            gripper_state_vector_[file_counter].push_back(ros::Time::now().toSec()) ;
            file_counter ++ ;
            ros::spinOnce() ;

        }

        string pack_path= ros::package::getPath("franka_LfD") ;
        string file_name= pack_path + "/data/gripper_state_demo.txt" ;
        general_utility::saveVectorMatrixToFile(file_name,gripper_state_vector_) ;

    }

    bool GripperControllerTrajectory::init(std::string trajectory_file) {
            
            GripperControllerBase::init() ;
            ROS_INFO("Following Reference Gripper Trajectory Node !!") ;
            string packPath = ros::package::getPath("franka_LfD"); 
            ROS_INFO("Gripper Node , Following Reference gripper Trajectory !!") ;
            
           if (general_utility::loadVectorMatrixFromFile(trajectory_file, 2,  ref_trajectory_) ==0 ) {
            return true ;
           }
           else{
            ROS_ERROR("Gripper Reference state file not found") ;
            return false ;
           }
           
       

    }

    void GripperControllerTrajectory::followReferenceTrajectory() {

        ros::Rate loop_rate(1000) ;
        int file_counter=0 ;
        int traj_size= ref_trajectory_.size() ;

        while (ros::ok() && file_counter<traj_size-1) {

            realtype des_width =  ref_trajectory_[file_counter][0] ; 
            realtype des_force =  ref_trajectory_[file_counter][1] ; 
           
           //change gripper state only if the current state does not match des. one
            if(! general_utility::equalcompare(des_width,commanded_width_) || ! general_utility::equalcompare(des_force,commanded_force_)) {
            GripperAction(commanded_width_ , commanded_force_) ;
            commanded_width_ =  des_width ;
            commanded_force_ =  des_force ;
            }
         
            act_trajectory_.push_back(std::vector<float>())  ;
            act_trajectory_[file_counter].push_back(commanded_width_) ;
            act_trajectory_[file_counter].push_back(commanded_force_) ;
            act_trajectory_[file_counter].push_back(ros::Time::now().toSec()) ;

            file_counter++ ;
            loop_rate.sleep() ;
        
            ros::spinOnce() ;

        }

        string path= ros::package::getPath("franka_LfD") + "/data/gripper_state_act.txt" ; 
        general_utility::saveVectorMatrixToFile(path,act_trajectory_) ;


    }


}
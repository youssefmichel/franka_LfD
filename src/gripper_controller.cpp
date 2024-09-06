#include <gripper_controller.h> 


namespace franka_LfD{ 

  

    
    bool GripperControllerBase::init() 
 
    {
    
    ros::Duration timeout(15.0) ;
    commanded_width_=0.07 ;
    commanded_force_ = 0.0 ;

    // if(MyGraspClient.waitForServer(timeout)) {
       
    //      bool homing_success=homingGripper() ;
    //      ROS_INFO("Gripper Started Successfully !!") ;
         
    //      return true ;
    // }
    // else {
    //     ROS_ERROR("Gripper could not be Started !!") ;
    //     return false ;
    // }
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
                commanded_width_ = 0.03 ;
                commanded_force_= 2.0 ;
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
           ROS_INFO("GRASP") ;
           
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
                ROS_WARN("Could Not Grasp Object !!") ;
                return false ;
             }

             }

        

     bool GripperControllerBase::ReleaseObject()  {
         
                commanded_width_ = 0.07 ;
                commanded_force_= 0.0 ;
                GripperAction(commanded_width_ , commanded_force_) ;
                return true ;
        
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

    bool GripperControllerTrajectory::init(std::string trajectory_file) {
            
            ROS_INFO("Following Reference Gripper Trajectory Node !!") ;
            string packPath = ros::package::getPath("franka_LfD"); 
            general_utility::loadVectorMatrixFromFile(trajectory_file, 2,  ref_trajectory_) ;

    }

    void GripperControllerTrajectory::followReferenceTrajectory() {

        ros::Rate loop_rate(1000) ;
        int file_counter=0 ;
        int traj_size= ref_trajectory_.size() ;

        while (ros::ok() && file_counter<traj_size-1) {

            commanded_width_ = ref_trajectory_[file_counter][0] ;
            commanded_force_ = ref_trajectory_[file_counter][1] ;

            GripperAction(commanded_width_ , commanded_force_) ;
            file_counter++ ;
            loop_rate.sleep() ;
            ros::spinOnce() ;

        }


    }


}
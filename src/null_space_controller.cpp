#include "null_space_controller.h"
#include <pluginlib/class_list_macros.h>

namespace franka_LfD{


bool NullSpaceController::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {  
    
    auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>() ;
    std::string arm_id ;

    if(! node_handle.getParam("arm_id",arm_id)) {

    ROS_ERROR_STREAM("CartesianImpedanceController: Could not read parameter arm_id");
    return false;
    }


     model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
    
    return true ;
                                
      } 




// Vec NullSpaceController::findDirection(const Vec &q, const Mat &J){

//      Eigen::FullPivLU<Mat> lu(J) ; 
//     Mat Z= lu.kernel() ; 
//     Vec q_right= q - Z.transpose()*0.01 ;
//     Vec q_left= q  + Z.transpose()*0.01 ;

//     if ()

// }



Vec NullSpaceController::findOptimalConfig(const Vec &q, const Mat &J) {

    int N_OF_ITER =100 ;
    Vec q_curr= q; 
    Vec q_opt =q_curr ;
    realtype dt=0.01 ;
    cout<<"q_opt start:  "<<q_opt.transpose()<<endl; 
    for (int k=0 ; k<N_OF_ITER ; k++ ) {

    Eigen::FullPivLU<Mat> lu(J) ; 
    Mat Z= lu.kernel() ; 
    Vec dq= q_curr-q_prev_ ;
    realtype manip_index_curr = computeManipIndex(q,J) ;
    realtype d_manip=manip_index_curr-manip_index_prev_ ;
    Vec d_manip_d_q = Vec::Zero(q.size()) ;

    for (int i=0 ; i<q.size() ; i++) {

        d_manip_d_q(i) = d_manip/dq(i) ;
    }
    
    Mat tmp = Z*Mat::Identity(q.size() , q.size() ) *Z.transpose() ; 
    Mat tmp_inv=tmp.inverse() ;
    Vec minimal_grad = tmp_inv * Z * d_manip_d_q.transpose() ; 
    q_opt = q_curr - Z.transpose() * minimal_grad *dt ;
    q_prev_=q_curr ;
    q_curr=q_opt ;
    manip_index_prev_ =manip_index_curr ;

    if(minimal_grad.norm()<0.001) {
        ROS_INFO("Optimal Config. Found !!") ;
        break ; 
    }

    cout<<"q_opt: "<<q_opt.transpose()<<endl; 
    cout << "manip index" << manip_index_prev_<<endl;

    }


    return q_opt ;



}

void NullSpaceController::update(const ros::Time&, const ros::Duration& period) {
 

}

void  NullSpaceController::stopping(const ros::Time&) {}


realtype NullSpaceController::computeManipIndex(const Vec &q, const Mat &J){
    
    
    Mat J_square= J*J.transpose()  ;
    return J_square.determinant()  ;
    
}

realtype NullSpaceController::computeManipIndex(const franka::RobotState &state){

std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector) ;
Eigen::Map<Eigen::Matrix<double, 6, 7>> J(jacobian_array.data()); 

Mat J_square= J*J.transpose() ;

return  J_square.determinant() ; ; 

// Mat J_square= J*J.transpose() ;
// return J_square.determinant() ;

}





}



PLUGINLIB_EXPORT_CLASS(franka_LfD::NullSpaceController,
                       controller_interface::ControllerBase)


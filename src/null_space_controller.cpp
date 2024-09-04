#include "null_space_controller.h"
#include <pluginlib/class_list_macros.h>

namespace franka_LfD{


NullSpaceController::NullSpaceController(std::unique_ptr<franka_hw::FrankaModelHandle> &model) {

    std::array<double, 42> jacobian_array = model->getZeroJacobian(franka::Frame::kEndEffector) ; 
    Eigen::Map<Eigen::Matrix<double, 6,7>> J(jacobian_array.data()) ;
    cout<<"Manip Index: "<<computeManipIndex(J)<<endl  ;
    curr_grad_direc_=gradDirection::LEFT ;
    Initial_point_optimal_flag_ = false; 
    ROS_INFO("Null Space Controller For Singularity Optimization") ;

}

NullSpaceController::NullSpaceController() {
    
}

franka::RobotState NullSpaceController::findDirection(std::unique_ptr<franka_hw::FrankaModelHandle> &model_handle,  franka::RobotState robot_state){
    
    //Find Initial Direction of minimization 
    double dt=0.04 ;
    Mat J= GetJacobianEigen(model_handle, robot_state ) ;
    Eigen::FullPivLU<Mat> lu(J) ; 
    Mat Z= lu.kernel() ; 
    Eigen::Map <Eigen::Matrix<double, 7,1>> q(robot_state.q.data()) ; 
    
    Vec q_right= q - Z*dt;
    Vec q_left= q  + Z*dt;
   

    franka::RobotState robot_state_right= robot_state ;
    franka::RobotState robot_state_left= robot_state ;
    cout <<"qRight: "<<q_right.transpose() <<endl ;
    cout <<"qLeft: "<<q_left.transpose() <<endl ;


    for (int i=0 ; i <7 ; i++) {

        robot_state_right.q[i] = q_right[i] ;
        robot_state_left.q[i] =  q_left[i] ;

    }

    Mat J_right= GetJacobianEigen (model_handle, robot_state_right) ;
    Mat J_left=  GetJacobianEigen (model_handle, robot_state_left) ;

    if( computeManipIndex(J_right) > computeManipIndex(J)  ) {

        curr_grad_direc_ = gradDirection::RIGHT ;
     
        return robot_state_right ;       

    }
    else if ( computeManipIndex(J_left) > computeManipIndex(J)  ){ 

        
        curr_grad_direc_= gradDirection::LEFT ;
        return robot_state_left ;
    }
    else {

        Initial_point_optimal_flag_ = true ;
        return robot_state ;
    }

}



Vec NullSpaceController::findOptimalConfig(std::unique_ptr<franka_hw::FrankaModelHandle> &model_handle, franka::RobotState robot_state) {

    int N_OF_ITER =500 ;
    
    Eigen::Map<Eigen::Matrix <double,7,1 > > q_curr(robot_state.q.data()) ;
    std::array<double, 42> jacobian_array = model_handle->getZeroJacobian( franka::Frame::kEndEffector, robot_state.q, robot_state.F_T_EE,robot_state.EE_T_K) ; 
    Eigen::Map<Eigen::Matrix<double,6,7> > Jac(jacobian_array .data()) ;
    manip_index_prev_= computeManipIndex(Jac) ;
    

    q_prev_ = q_curr ;
    franka::RobotState curr_robot_state_grad= findDirection(model_handle, robot_state) ;
    new (&q_curr) Eigen::Map<Eigen::Matrix <double,7,1 >>(curr_robot_state_grad.q.data());

    Mat J =GetJacobianEigen(model_handle , curr_robot_state_grad) ;
    float manip_index_curr = computeManipIndex( J) ;
    Vec q_opt =q_curr ;
    realtype dt=0.01 ;

    if(Initial_point_optimal_flag_) {
        return q_opt ;
    }

    realtype d_manip=(manip_index_curr-manip_index_prev_) / dt ;
    realtype d_manip_prev = d_manip ;
   
    for (int k=0 ; k<N_OF_ITER ; k++ ) {

    Eigen::FullPivLU<Mat> lu(J) ; 
    Mat Z= lu.kernel() ; 
    Vec dq= q_curr-q_prev_ ;
 
    Mat tmp = Z*Mat::Identity(q_curr.size() , q_curr.size() ) *Z.transpose() ; 
    Mat tmp_inv=tmp.inverse() ;
    Vec minimal_grad = tmp_inv * Z * d_manip; 
     
    q_opt = q_curr +  curr_grad_direc_* Z *dt ;
    q_prev_=q_curr ;
    q_curr=q_opt ;
    manip_index_prev_ =manip_index_curr ;

    for (int i=0 ; i <7 ; i++) {

        curr_robot_state_grad.q[i] = q_curr[i] ;
    }
    
    Mat J =GetJacobianEigen(model_handle , curr_robot_state_grad ) ;
    manip_index_curr = computeManipIndex( J) ;
    d_manip_prev=d_manip ;
    d_manip=(manip_index_curr-manip_index_prev_) / dt ;

       if( general_utility::sign(d_manip ) !=  general_utility::sign(d_manip_prev )   ) {

        return q_opt ;
        
    }

    

    }



    return q_opt ;

}




float NullSpaceController::computeManipIndex( const Mat &J){
    
    
    Mat J_square= J*J.transpose()  ;
    return J_square.determinant()  ;
    
}

Mat NullSpaceController::GetJacobianEigen(std::unique_ptr<franka_hw::FrankaModelHandle> &model_handle, franka::RobotState robot_state) {


    std::array<double, 42> jac_array= model_handle->getZeroJacobian(franka::Frame::kEndEffector, robot_state.q, robot_state.F_T_EE, robot_state.EE_T_K  ) ;
    Eigen::Map<Eigen::Matrix<double, 6,7>> Jac(jac_array.data()) ;

    return Jac ;

}





}



// PLUGINLIB_EXPORT_CLASS(franka_LfD::NullSpaceController,
//                        controller_interface::ControllerBase)


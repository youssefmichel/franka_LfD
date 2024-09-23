# pragma once 

#include <stdlib.h>
#include <fstream> 
#include <vector>
#include "eigen3/Eigen/Dense"
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/Core>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <ros/ros.h>
#include <string.h>

using namespace std ;
typedef double realtype ;
typedef Eigen::Matrix<realtype, Eigen::Dynamic, Eigen::Dynamic> Mat;
typedef Eigen::Matrix<realtype, Eigen::Dynamic, 1> Vec;

#ifndef eps
#define eps 1.0e-12
#endif

/**
 * @class SkillPublisher 
 * @brief publishes the learnt skill to the robot
 * 
 * useful utlity functions 
 */



namespace general_utility{

    int loadVectorMatrixFromFile (std::string fileName, int cols, vector<vector<float>> &outMat) ; 
    void saveVectorMatrixToFile (string fileName, vector < vector <float> > outMat) ;
    template <typename T> int sign(T value) {
                            
    if (value > 0) {
        return 1;   // Positive number
    } else if (value < 0) {
        return -1;  // Negative number
    } else {
        return 0;   // Zero
    }
}

    template <typename U,typename V> 
    bool equalcompare(U a, V b){
            float epsilon = 0.00000001 ;
            return std::abs(a - b) < epsilon;
    }
}

namespace quat_utils{ 

    int quat_conjugate(double q1[], double q2[]);


int quat_mult(double q1[], double q2[], double q[])
// Quaternion multiplication
;

int quat_exp(double omega[], double q[])
//Quaternion exponential
;


int quat_log(double q1[], double q2[], double log_q[])
//Calculates logarithm of orientation difference between quaternions
;


int normalize(double q[], double qnew[])
;


int vnorm(double v[], double& vnorm)
;



int qnorm(double q[], double& qnorm) ;

}
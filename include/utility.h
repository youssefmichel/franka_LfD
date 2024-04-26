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



namespace general_utility{

    int loadVectorMatrixFromFile (std::string fileName, int cols, vector<vector<float>> &outMat) ; 
    void saveVectorMatrixToFile (string fileName, vector < vector <float> > outMat) ;



}
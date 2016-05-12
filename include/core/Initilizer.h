//
// Created by patrick on 5/10/16.
//

#ifndef MAST_PROJECT_INITILIZER_H
#define MAST_PROJECT_INITILIZER_H


#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <Eigen/Dense>
#include "util/quat_ops.h"


using namespace std;
using namespace cv;

namespace MAST {

    class Initilizer {


    public:
        Eigen::Matrix3d calib_cam_K;

        Initilizer();
        void testing();

        /*
         * This function takes the corresponding img and soanr measurement of a feature, the relative rotation and translation between Camera and Sonar,
         * and will return the local 3D positions of this feature in camera frame
         *
         * init_feat_cam1: use all the constraints from the camera and sonar
         * init_feat_cam2: use a planar triangular method to solve the depth of camera
         *
         * cv:: point pt_img            image u v
         * cv:: point pt_sonar          sonar r th
         * Eigen::Matrix<double,4,1>    quaternion c to s
         * Eigen::matrix<double,3,1>    position c in s
         */
        Eigen::Matrix<double, 3, 1> init_feat_cam1(cv::Point pt_img, cv::Point pt_sonar, Eigen::Matrix<double,4,1> q_c_to_s, Eigen::Matrix<double,3,1> p_c_in_s);
        Eigen::Matrix<double, 3, 1> init_feat_cam2(cv::Point pt_img, cv::Point pt_sonar, Eigen::Matrix<double,4,1> q_c_to_s, Eigen::Matrix<double,3,1> p_c_in_s);

        /*
         * This function takes 3 corresponding feature points in camera frame k and camera frame k+1
         * and will return the quaternion k+1 to k and translation k+1 in k.
         *
         * Eigen::Matrix<double, 3, 3> feat_cam_k       3 3d feature positions
         * Eigen::Matrix<double, 3, 3> feat_cam_k+1     3 3d feature positions
         */
        Eigen::Matrix<double, 7, 1> init_cam_pose(Eigen::Matrix<double, 3, 3> feat_cam_k, Eigen::Matrix<double, 3, 3> feat_cam_k_plus_1);
    };



}


#endif //MAST_PROJECT_INITILIZER_H

//
// Created by patrick on 5/10/16.
//

#include "core/Initilizer.h"

using namespace MAST;


void Initilizer::testing() {
    cout << "Hello World" << endl;
}

Initilizer::Initilizer() {
    Eigen::Matrix<double,3,3> K;
    K.setZero();

    K(0,0) = 466.6599239525347;

    K(0,2) = 408.15836612911477;

    K(1,1) = 465.8386070513852;
    K(1,2) = 243.27181754452832;

    K(2,2) = 1.0;
    calib_cam_K =K;
}

Eigen::Matrix<double, 3, 1> Initilizer::init_feat_cam1(cv::Point pt_img,
                                                    cv::Point pt_sonar,
                                                    Eigen::Matrix<double, 4, 1> q_c_to_s,
                                                    Eigen::Matrix<double, 3, 1> p_c_in_s) {

    //Normalize the uv coordinate of the camera feature points with the camera intrinsic K
    double u_normalized = (pt_img.x - calib_cam_K(0,2))/calib_cam_K(0,0);
    double v_normalized = (pt_img.y - calib_cam_K(1,2))/calib_cam_K(1,1);

    //This is also the bearing of the feature
    Eigen::Matrix<double, 3, 1> bearing_f_c;
    bearing_f_c << u_normalized,v_normalized,1;
    bearing_f_c = bearing_f_c / bearing_f_c.norm();

    //Find the orthogonal vector to the bearing of feature point in camera
    Eigen::Matrix<double, 3, 1> bearing_perpendicular1;
    bearing_perpendicular1 << 1,0, -u_normalized;
    Eigen::Matrix<double, 3, 1> bearing_perpendicular2;
    bearing_perpendicular2 = bearing_perpendicular1.cross(bearing_f_c);

    Eigen::Matrix<double, 3, 3> R_C_to_S = quat_2_Rot(q_c_to_s);

    //Construct the linear equation to solve the bearing_f_s
    double r_sonar = pt_sonar.x;
    double th_sonar = pt_sonar.y;

    Eigen::Matrix<double, 3, 3> linear_M_left;
    linear_M_left.block(0,0,1,3) = r_sonar * bearing_perpendicular1.transpose() * R_C_to_S.transpose();
    linear_M_left.block(1,0,1,3) = r_sonar * bearing_perpendicular2.transpose() * R_C_to_S.transpose();
    linear_M_left.block(2,0,1,3) << sin(th_sonar), cos(th_sonar), 0;

    Eigen::Matrix<double, 3, 1> linear_V_right;
    linear_V_right(0,0) = bearing_perpendicular1.transpose() * R_C_to_S.transpose() * p_c_in_s;
    linear_V_right(1,0) = bearing_perpendicular2.transpose() * R_C_to_S.transpose() * p_c_in_s;
    linear_V_right(2,0) = 0;

    Eigen::Matrix<double, 3, 1> bearing_f_s;
    bearing_f_s = linear_M_left.inverse() * linear_V_right;

    if (bearing_f_s.norm() != 1){
        bearing_f_s = bearing_f_s / bearing_f_s.norm();
    }

    //Return the 3D feature position in camera frame
    Eigen::Matrix<double, 3, 1> feat_init;
    feat_init = r_sonar * R_C_to_S.transpose() * bearing_f_s - R_C_to_S.transpose() * p_c_in_s;

    return feat_init;

}

Eigen::Matrix<double, 3, 1> Initilizer::init_feat_cam2(cv::Point pt_img,
                                           cv::Point pt_sonar,
                                           Eigen::Matrix<double,4,1> q_c_to_s,
                                           Eigen::Matrix<double,3,1> p_c_in_s){
    //Normalize the uv coordinate of the camera feature points with the camera intrinsic K
    double u_normalized = (pt_img.x - calib_cam_K(0,2))/calib_cam_K(0,0);
    double v_normalized = (pt_img.y - calib_cam_K(1,2))/calib_cam_K(1,1);

    //This is also the bearing of the feature
    Eigen::Matrix<double, 3, 1> bearing_f_c;
    bearing_f_c << u_normalized,v_normalized,1;
    bearing_f_c = bearing_f_c / bearing_f_c.norm();

    Eigen::Matrix<double, 3, 3> R_C_to_S = quat_2_Rot(q_c_to_s);

    Eigen::Matrix<double, 3, 1> p_s_in_c = R_C_to_S.transpose() * p_c_in_s;

    //Construct the linear equation to solve the bearing_f_s
    double r_sonar = pt_sonar.x;
    double th_sonar = pt_sonar.y;

    //Triangular solution
    double dot_prod_bearing_and_psc = bearing_f_c.transpose() * p_s_in_c;
    double norm_p_s_in_c = p_s_in_c.norm();
    double depth_cam = dot_prod_bearing_and_psc + sqrt(pow(dot_prod_bearing_and_psc,2)+ pow(r_sonar,2)-pow(norm_p_s_in_c,2));
    Eigen::Matrix<double, 3, 1> feat_init = depth_cam * bearing_f_c;

    return feat_init;

};

Eigen::Matrix<double, 7, 1> Initilizer::init_cam_pose(Eigen::Matrix<double, 3, 3> feat_cam_k,
                                          Eigen::Matrix<double, 3, 3> feat_cam_k_plus_1){


    Eigen::Matrix<double, 3, 1> feat1_cam_k = feat_cam_k.block(0,0,1,3).transpose();
    Eigen::Matrix<double, 3, 1> feat2_cam_k = feat_cam_k.block(1,0,1,3).transpose();
    Eigen::Matrix<double, 3, 1> feat3_cam_k = feat_cam_k.block(2,0,1,3).transpose();

    Eigen::Matrix<double, 3, 1> delta_12_cam_k = feat1_cam_k - feat2_cam_k;
    Eigen::Matrix<double, 3, 1> delta_23_cam_k = feat2_cam_k - feat3_cam_k;
    Eigen::Matrix<double, 3, 1> delta_cross_cam_k = delta_12_cam_k.cross(delta_23_cam_k);

    Eigen::Matrix<double, 3, 1> feat1_cam_k_plus_1 = feat_cam_k_plus_1.block(0,0,1,3).transpose();
    Eigen::Matrix<double, 3, 1> feat2_cam_k_plus_1 = feat_cam_k_plus_1.block(1,0,1,3).transpose();
    Eigen::Matrix<double, 3, 1> feat3_cam_k_plus_1 = feat_cam_k_plus_1.block(2,0,1,3).transpose();

    Eigen::Matrix<double, 3, 1> delta_12_cam_k_plus_1 = feat1_cam_k_plus_1 - feat2_cam_k_plus_1;
    Eigen::Matrix<double, 3, 1> delta_23_cam_k_plus_1 = feat2_cam_k_plus_1 - feat3_cam_k_plus_1;
    Eigen::Matrix<double, 3, 1> delta_cross_cam_k_plus_1 = delta_12_cam_k_plus_1.cross(delta_23_cam_k_plus_1);

    Eigen::Matrix<double, 3, 3> delta_M_cam_k;
    delta_M_cam_k.block(0,0,3,1) = delta_12_cam_k;
    delta_M_cam_k.block(0,1,3,1) = delta_23_cam_k;
    delta_M_cam_k.block(0,2,3,1) = delta_cross_cam_k;

    Eigen::Matrix<double, 3, 3> delta_M_cam_k_plus_1;
    delta_M_cam_k_plus_1.block(0,0,3,1) = delta_12_cam_k_plus_1;
    delta_M_cam_k_plus_1.block(0,1,3,1) = delta_23_cam_k_plus_1;
    delta_M_cam_k_plus_1.block(0,2,3,1) = delta_cross_cam_k_plus_1;

    Eigen::Matrix<double, 3, 3> R_k_plus_1_to_k = delta_M_cam_k * delta_M_cam_k_plus_1.inverse();
    Eigen::Matrix<double, 3, 1> p_k_plus_1_in_k = (feat1_cam_k + feat2_cam_k + feat3_cam_k) - R_k_plus_1_to_k * (feat1_cam_k_plus_1 + feat2_cam_k_plus_1 + feat3_cam_k_plus_1);
    p_k_plus_1_in_k = p_k_plus_1_in_k/3;

    Eigen::Matrix<double, 7, 1> cam_pose_k_k_plus_1;
    cam_pose_k_k_plus_1.block(0,0,4,1) = rot_2_quat(R_k_plus_1_to_k);
    cam_pose_k_k_plus_1.block(4,0,3,1) = p_k_plus_1_in_k;

    return cam_pose_k_k_plus_1;

};
//
// Created by keck on 5/4/16.
//
#pragma once

#ifndef MAST_PROJECT_FRAME_CLASSES_H
#define MAST_PROJECT_FRAME_CLASSES_H

#endif //MAST_PROJECT_FRAME_CLASSES_H


#include <iostream>
#include <ctime>
#include <cstdlib>
#include "../types/JPL7.h"
#include <Eigen/Dense>

using namespace std;


namespace MAST {

    class Camera;

    class Feature_Class {

    public:
        Feature_Class();

        //vector of pixel coordinates that the feature is seen in. If it is not seen in a given image, then the pair is -1,-1
        vector<pair<int, int> > uv_locations;

        //vector of pixel coordinates that the feature is seen in. If it is not seen in a given image, then the pair is -1,-1
        vector<pair<double, double> > r_theta_values;

        //List of cameras it has been seen in
        vector<Camera* > as_seen_in;

        //True position in the global frame;
        Eigen::Matrix<double, 3, 1> true_position;

        Feature *feature;


    };

    class Camera {

    public:

        JPL7 *pose;

        Camera();

        int id;

        int width;

        int height;

        Eigen::Matrix<double, 3, 3> K;

        Eigen::Matrix<double, 3, 3> Kinv;

        vector<Feature_Class *> feature_list;

        Eigen::Matrix<double, 3, 3> R_G_to_C;

        Eigen::Matrix<double, 3, 1> p_G_to_C;

        Eigen::Matrix<double, 3, 3> R_C_to_S;

        Eigen::Matrix<double, 3, 1> p_c_in_s;

        double r_max;

        double th_max;

        double psi_max;


        JPL7 *camera_vertex;

        double addNoise(double value, double sigma_sq) {
            double corrupted_value = value;
            //Corrupt measurements here
            return corrupted_value;
        }

        void project_point(Feature_Class *feat) {
            Eigen::Matrix<double, 3, 1> p_in_c = R_G_to_C * (feat->true_position - p_G_to_C);
            pair<int, int> uv;
            uv.first = -1;
            uv.second = -1;

            double sigma_sq;

            if (p_in_c(2, 0) > 3) {

                Eigen::Matrix<double, 3, 1> p_hom = (p_in_c) / (p_in_c(2, 0));
                Eigen::Matrix<double, 3, 1> pixel = K * p_hom;

                if ((pixel(0, 0) >= 0) && (pixel(0, 0) < width) && (pixel(1, 0) >= 0) && (pixel(1, 0) < height)) {
                    uv.first = (int) addNoise(pixel(0, 0), sigma_sq);
                    uv.second = (int) addNoise(pixel(0, 0), sigma_sq);
                    feat->as_seen_in.push_back(this);
                }

                feat->uv_locations.push_back(uv);

            }

        }

        void project_point_to_Sonar(Feature_Class *feat) {

            pair<double, double> r_theta;

            r_theta.first = -1;
            r_theta.second = -1;

            Eigen::Matrix<double, 3, 1> p_in_s = R_C_to_S * R_G_to_C * (feat->true_position - p_G_to_C) + p_c_in_s;

            double r = p_in_s.norm();

            double theta = atan2(p_in_s(1, 0), p_in_s(0, 0));

            double psi = atan2(p_in_s(2, 0), sqrt(pow(p_in_s(1, 0), 2) + pow(p_in_s(1, 0), 2)));

            if ((abs(r) < r_max) && (abs(theta) < th_max) && (abs(psi) < psi_max)) {
                r_theta.first = r;
                r_theta.second = theta;
                feat->r_theta_values.push_back(r_theta);
            }


        }


    };

}
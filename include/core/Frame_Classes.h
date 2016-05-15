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

        bool initialized;

        //vector of pixel coordinates that the feature is seen in. If it is not seen in a given image, then the pair is -1,-1
        vector<pair<int, int> > uv_locations;

        //vector of pixel coordinates that the feature is seen in. If it is not seen in a given image, then the pair is -1,-1
        vector<pair<double, double> > r_theta_values;

        //List of cameras it has been seen in
        vector<Camera* > as_seen_in;

        //True position in the global frame;
        Eigen::Matrix<double, 3, 1> true_position;

        Feature* feature;

        vector<Camera*> seen_from_image;

        vector<Camera*> seen_from_image_and_sonar;


    };

    class Camera {

    public:

        bool initialized;

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

            //double sigma_sq;

            if (p_in_c(2, 0) > 0) {

                Eigen::Matrix<double, 3, 1> p_hom = (p_in_c) / (p_in_c(2, 0));
                Eigen::Matrix<double, 3, 1> pixel = K * p_hom;

                int xn= rand() %5;

                bool xnsign= rand() %2;

                int yn= rand() %5;

                bool ynsign= rand() %2;

                if (xnsign) xn=-1*xn;

                if (ynsign) yn=-1*yn;

                /*cout << "xn- " << xn <<  endl;

                cout << "yn- " << yn  << endl;*/

                int un=  pixel(0, 0)+xn;

                int vn= pixel(1,0)+yn;


                if ((pixel(0, 0) >= 0) && (pixel(0, 0) < width) && (pixel(1, 0) >= 0) && (pixel(1, 0) < height)) {
                    //cout << "They should be added" << endl;
                    uv.first = un;
                    uv.second = vn;
                    feat->as_seen_in.push_back(this);
                    feature_list.push_back(feat);
                }

                feat->uv_locations.push_back(uv);

            }

        }

        void project_point_to_Sonar(Feature_Class *feat) {

            pair<double, double> r_theta;

            r_theta.first = -1;
            r_theta.second = -1;

            Eigen::Matrix<double, 3, 1> p_in_s = R_C_to_S * R_G_to_C * (feat->true_position - p_G_to_C) + p_c_in_s;

            /*cout << p_in_s << endl << endl;

            cout << feat->true_position << endl << endl;

            cout << p_G_to_C << endl << endl;

            cout << p_c_in_s << endl << endl;*/


            double r = p_in_s.norm();

            double theta = atan2(p_in_s(1, 0), p_in_s(0, 0));




            double rn= rand() %5;

            bool rnsign= rand() %2;

            double thn=  (((float) rand()) / (float) RAND_MAX);

            bool thnsign= rand() %2;


            rn = rn + (((float) rand()) / (float) RAND_MAX);

            rn= rn/50;


            cout << "RN" << endl << rn << endl << endl;

            if (rnsign) rn=-1*rn;

            if (thnsign) thn=-1*thn;



            double rc=  r+rn;

            double thc= theta+thn;


            double psi = atan2(p_in_s(2, 0), (sqrt(pow(p_in_s(0, 0), 2) + pow(p_in_s(1, 0), 2))));

            /*cout << r << " , " << theta << " , " << psi << endl << endl;

            std::exit(1);*/

            if ((abs(r) < r_max) && (abs(theta) < th_max) && (abs(psi) < psi_max)) {
                r_theta.first = rc;
                r_theta.second = thc;
                if (feat->uv_locations[this->id].first == -1){
                    feature_list.push_back(feat);

                }

            }
            feat->r_theta_values.push_back(r_theta);


        }


    };

}
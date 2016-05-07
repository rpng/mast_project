//
// Created by keck on 5/4/16.
//
#pragma once
#ifndef ROBUST_IMAGESIM_H
#define ROBUST_IMAGESIM_H

#endif //ROBUST_IMAGESIM_H
#ifndef ROBUST_IMAGESIM_H
#define ROBUST_IMAGESIM_H

#endif //ROBUST_IMAGESIM_H

#include <iostream>
#include <ctime>
#include <cstdlib>
#include "core/Frame_Classes.h"
#include <Eigen/Dense>

using namespace std;

namespace MAST {

//vector<array<double, 200> > ImageSim()
    void ImageSim(vector<Camera *> cam_vec, int Feat_number) {
        srand(time(NULL));

        float x, y, z;
        bool xsign, ysign, zsign; // true if negative


        for (int i = 0; i < Feat_number; i++) {
            x = rand() % 20;
            y = rand() % 20;
            z = rand() % 20;

//cout << (((float) rand()) / (float) RAND_MAX) << endl;

            x = x + (((float) rand()) / (float) RAND_MAX);
            y = y + (((float) rand()) / (float) RAND_MAX);
            z = z + (((float) rand()) / (float) RAND_MAX);

            xsign = rand() % 2;
            ysign = rand() % 2;
            zsign = rand() % 2;

            if (xsign) x *= -1;
            if (ysign) y *= -1;
            if (zsign) z *= -1;

            Eigen::Matrix<double, 3, 1> true_p;

            true_p << x, y, z;

            Feature_Class *feat = new Feature_Class();

            feat->true_position = true_p;

            for (size_t c = 0; c < cam_vec.size(); c++) {
                cam_vec[c]->project_point(feat);
                cam_vec[c]->project_point_to_Sonar(feat);
            }


        }
    }

    vector<Camera *> CamAsRandomizedPolygonSim(int num_cams, double side_length, double start_x, double start_y) {

        vector<Camera *> cam_vec;

        srand(time(NULL));

        double theta = 0;

        double x = start_x;

        double y = start_y;

        double z = rand() % 20;

        bool zsign = rand() % 2;

        z = z + (((float) rand()) / (float) RAND_MAX);

        if (zsign) z *= -1;






        //Blah blah get stuff

        int n = 0;

        while (n < num_cams) {


            Eigen::Matrix<double, 3, 3> Rn_z;

            double r = sqrt(pow(x, 2) + pow(y, 2));


            Rn_z << cos(theta), sin(theta), 0,
                    -sin(theta), cos(theta), 0,
                    0, 0, 1;

            double psi = atan2(z, r);

            Eigen::Matrix<double, 3, 3> Rn_x;

            Rn_x << 1, 0, 0,
                    0, cos(psi), sin(psi),
                    0, -sin(psi), cos(psi);

            Eigen::Matrix<double, 3, 3> R_nom;

            R_nom = Rn_x * Rn_z;

            double eps_z;

            double eps_x;

            double eps_y;


            eps_x = rand() % 4;

            eps_y = rand() % 4;

            eps_z = rand() % 4;


            eps_x = eps_x + (((float) rand()) / (float) RAND_MAX);
            eps_y = eps_y + (((float) rand()) / (float) RAND_MAX);
            eps_z = eps_z + (((float) rand()) / (float) RAND_MAX);

            //This should put the error to be from -25 to 25 degrees

            bool exsign = rand() % 2;

            bool eysign = rand() % 2;

            bool ezsign = rand() % 2;

            eps_x = eps_x / 10;
            eps_y = eps_y / 10;
            eps_z = eps_z / 10;

            if (exsign) eps_x *= -1;
            if (eysign) eps_y *= -1;
            if (ezsign) eps_z *= -1;

            Eigen::Matrix<double, 3, 3> Re_x;
            Eigen::Matrix<double, 3, 3> Re_y;
            Eigen::Matrix<double, 3, 3> Re_z;

            Re_x << 1, 0, 0,
                    0, cos(eps_x), sin(eps_x),
                    0, -sin(eps_x), cos(eps_x);


            Re_y << cos(eps_y), 0, -sin(eps_y),
                    0, 1, 0,
                    sin(eps_y), 0, cos(eps_y);

            Re_z << cos(eps_z), sin(eps_z), 0,
                    -sin(eps_z), cos(eps_z), 0,
                    0, 0, 1;


            Eigen::Matrix<double, 3, 3> Rtrue = Re_x * Re_y * Re_z * R_nom;


            //Create camera

            Camera *cam_i = new Camera();

            Eigen::Matrix<double, 3, 1> p_c_in_g;

            p_c_in_g << x, y, z;

            cam_i->R_G_to_C = Rtrue;

            cam_i->p_G_to_C = p_c_in_g;

            cam_i->id = n;

            cam_vec.push_back(cam_i);



            //Start next camera

            theta += M_PI_2 - (num_cams - 2) * (M_PI) / num_cams;

            double zplus = rand() % 20 + (((float) rand()) / (float) RAND_MAX);

            bool zplussign = rand() % 2;

            if (zplussign) zplus *= -1;

            z = z + zplus;

            x += side_length * cos(theta);

            y += side_length * sin(theta);


            n++;


        }

        return cam_vec;


    }

}
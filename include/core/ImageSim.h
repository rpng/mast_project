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
//#include "core/Frame_Classes.h"
#include <Eigen/Dense>

using namespace std;

namespace MAST {

    class Camera;

//vector<array<double, 200> > ImageSim()
    void ImageSim(vector<Camera *> cam_vec, int Feat_number) ;

    vector<Camera *> CamAsRandomizedPolygonSim(int num_cams, double side_length, double start_x, double start_y);

}
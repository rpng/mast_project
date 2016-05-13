//
// Created by keck on 5/3/16.
//
#pragma once
#ifndef MAST_PROJECT_SFMGRAPH_H
#define MAST_PROJECT_SFMGRAPH_H


#include <g2o/core/sparse_optimizer.h>
#include "opencv2/opencv.hpp"
#include "core/Corresponder.h"
#include "types/JPL7.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "Frame_Classes.h"
#include <Eigen/Dense>

namespace MAST {
    class SFMgraph {


    public:

        SFMgraph();

        g2o::SparseOptimizer graph;

        void AddImageandSonarEdges(JPL7* Camera_vertex, JPL7* Calib, cv::Mat Image, cv::Mat Sonar,
                                   vector <Point> Image_Features, vector<Feature*> Feature_list);

        void AddImageEdges(JPL7* Camera_vertex, cv::Mat Image, vector <Point> Image_Features,
                           vector<Feature*> Feature_list);


        //Finds initial position from two "known" camera frames
        Eigen::Matrix<double,3,1> triangulate_point (JPL7 *Camera_1, JPL7 *Camera_2, Point2d uv1, Point2d uv2);


        //Finds camera position from known feature positions with realtive position measurements

        void find_camera_from_features (JPL7* Camera_1, vector<Eigen::Matrix<double,3,1>> uv, vector<Feature_Class*> Feature_list);

        Eigen::Matrix<double,3,1> find_feature_from_image_and_sonar (JPL7* Camera, JPL7* Calib, Eigen::Matrix<double,3,1> uv, double r);




    };

}

#endif //MAST_PROJECT_SFMGRAPH_H
//
// Created by keck on 5/3/16.
//

#ifndef MAST_PROJECT_SFMGRAPH_H
#define MAST_PROJECT_SFMGRAPH_H

#endif //MAST_PROJECT_SFMGRAPH_H
#pragma once

#include <g2o/core/sparse_optimizer.h>
#include "opencv2/opencv.hpp"
#include "Corresponder.h"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

namespace MAST {
    class SFMgraph {


    public:

        SFMgraph();

        g2o::SparseOptimizer graph;

        void AddImageandSonarEdges(JPL7 *Camera_vertex, JPL7 *Calib, cv::Mat Image, cv::Mat Sonar,
                                   vector <Point> Image_Features, vector<Feature *> Feature_list);

        void AddImageEdges(JPL7 *Camera_vertex, cv::Mat Image, vector <Point> Image_Features,
                           vector<Feature *> Feature_list);


    };

}
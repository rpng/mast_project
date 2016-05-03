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
#include "SFMgraph.h"
#include "types/JPL7.h"
#include <Eigen/Dense>

using namespace std;
using namespace MAST;

SFMgraph::SFMgraph() {
}


void SFMgraph::AddImageandSonarEdges(JPL7* Camera_vertex, JPL7* Calib, cv::Mat Image, cv::Mat Sonar, vector<Point> Image_Features, vector<Feature*> Feature_list){
    vector< pair<double,double> > r_theta = Find_Correspondences(Image, Sonar, Image_Features);
    for (size_t i=0; i < r_theta.size(); i++){

        if (r_theta[i].first !=Infinity) {

            AddImageEdges( Camera_vertex, Image, Image_Features, Feature_list)


            //Add Sonar Edge


            SonarEdge* s_edge = new SonarEdge();

            s_edge->setVertex(0, Calib);

            s_edge->setVertex(1, Camera_vertex);

            s_edge->setVertex(2, Feature_list[i]);

            Eigen::Matrix<double, 2, 1> Meas_sonar;

            Meas_sonar << r_theta[i].first, r_theta[i].second;

            s_edge->setMeasurement(Meas_sonar);

            //Just a placeholder for the true Info
            Eigen::Matrix<double, 2,2 > Info_S = Eigen::MatrixXd::Identity(2,2);

            s_edge->setInformation(Info_S);

            this->graph->addEdge(s_edge);

        }
    }
}

void SFMgraph::AddImageEdges(JPL7* Camera_vertex,  cv::Mat Image, vector<Point> Image_Features, vector<Feature*> Feature_list){
    //Add Image Edge
    ImageEdge* i_edge= new ImageEdge();
    i_edge->setVertex(0,Camera_vertex);
    i_edge->setVertex(1,Feature_list[i]);


    Eigen::Matrix<double, 2, 1> Meas;
    Meas << Image_Features[i].x, Image_Features[i].y;
    i_edge->setMeasurement(Meas);

    //Just a placeholder for the true Info
    Eigen::Matrix<double, 2,2 > Info = Eigen::MatrixXd::Identity(2,2);

    i_edge->setInformation(Info);

    this->graph->addEdge(i_edge);
}
#pragma once
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <Eigen/Dense>

using namespace std;
using namespace cv;

namespace MAST {

    void CallBackFunc(int event, int x, int y, int flags, void *param);


    Point Correspondence(Mat Image, Mat Sonar);

    pair<double, double> sonar_meas_from_pt(Point sonar_pt);


    vector<pair<double, double>> Find_Correspondences(Mat Image, Mat Sonar, vector<Point> Image_Features);

}
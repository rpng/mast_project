//
// Created by keck on 5/7/16.
//
#include "core/Corresponder.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <Eigen/Dense>


using namespace std;
using namespace cv;

namespace MAST {

    void CallBackFunc(int event, int x, int y, int flags, void *param) {
        if (event == CV_EVENT_LBUTTONDOWN) {
            cv::Point *ptPtr = (cv::Point *) param;
            ptPtr->x = x;
            ptPtr->y = y;
        }

        if (event == CV_EVENT_RBUTTONDOWN) {
            cv::Point *ptPtr = (cv::Point *) param;
            ptPtr->x = Eigen::Infinity;
            ptPtr->y = Eigen::Infinity;
        }

    }


    Point Correspondence(Mat Image, Mat Sonar) {

        cv::Point2i ptS(-1, -1);

        //Create a window
        namedWindow("Image Window", 1);

        namedWindow("Sonar Window", 1);

        //set the callback function for any mouse event

        setMouseCallback("Sonar Window", CallBackFunc, (void *) &ptS);

        //show the image
        imshow("Image Window", Image);

        // Wait until user press some key

        imshow("Sonar Window", Sonar);

        waitKey(0);

        return ptS;

    }

    pair<double, double> sonar_meas_from_pt(cv::Point sonar_pt) {
        pair<double, double> r_theta;
        //Here I convert from x,y in sonar image to r and theta
        return r_theta;
    }


    vector<pair<double, double>> Find_Correspondences(Mat Image, Mat Sonar, vector<Point> Image_Features) {
        vector<pair<double, double>> sonar_measurements;
        Scalar color(0, 0, 255);

        for (size_t i = 0; i < Image_Features.size(); i++) {
            Mat Image_copy = Image;
            cv::drawMarker(Image_copy, Image_Features[i], MARKER_CROSS, 20, 1, 8);
            Point Sonar_Reading = Correspondence(Image_copy, Sonar);
            pair<double, double> sonar_meas = sonar_meas_from_pt(Sonar_Reading);
            sonar_measurements.push_back(sonar_meas);

        }

    }

}
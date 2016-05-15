//
// Created by keck on 5/3/16.
//

#include <g2o/core/sparse_optimizer.h>

#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
//#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/robust_kernel_impl.h>
//#include <g2o/core/estimate_propagator.h>
//#include <g2o/core/sparse_optimizer_terminate_action.h>
#include "opencv2/opencv.hpp"
#include "core/Corresponder.h"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "core/SFMgraph.h"
#include "types/JPL7.h"
#include <Eigen/Dense>
#include "core/Frame_Classes.h"

#include "util/quat_ops.h"

#include <unsupported/Eigen/MatrixFunctions>

using namespace std;

namespace MAST {

    SFMgraph::SFMgraph() {
        typedef g2o::BlockSolverX BlockSolver;
        typedef g2o::LinearSolverCSparse<BlockSolver::PoseMatrixType> LinearSolver;
        //typedef g2o::LinearSolverPCG<BlockSolver::PoseMatrixType> LinearSolver;
        LinearSolver* solver = new LinearSolver();
        BlockSolver* blockSolver = new BlockSolver(solver);
        g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg(blockSolver);
        this->graph.setAlgorithm(algorithm);

        this->graph.setVerbose(false); // printOptimizationInfo
        solver->setWriteDebug(false);
        blockSolver->setWriteDebug(false);
        algorithm->setWriteDebug(false);
    }


    void SFMgraph::AddImageandSonarEdges(JPL7 *Camera_vertex, JPL7 *Calib, cv::Mat Image, cv::Mat Sonar,
                                         vector<Point> Image_Features, vector<Feature *> Feature_list) {
        vector<pair<double, double> > r_theta = Find_Correspondences(Image, Sonar, Image_Features);
        for (size_t i = 0; i < r_theta.size(); i++) {

            if (r_theta[i].first != Eigen::Infinity) {

                AddImageEdges(Camera_vertex, Image, Image_Features, Feature_list);


                //Add Sonar Edge


                SonarEdge *s_edge = new SonarEdge();

                s_edge->setVertex(0, Calib);

                s_edge->setVertex(1, Camera_vertex);

                s_edge->setVertex(2, Feature_list[i]);

                Eigen::Matrix<double, 2, 1> Meas_sonar;

                Meas_sonar << r_theta[i].first, r_theta[i].second;

                s_edge->setMeasurement(Meas_sonar);

                //Just a placeholder for the true Info
                Eigen::Matrix<double, 2, 2> Info_S = Eigen::MatrixXd::Identity(2, 2);

                s_edge->setInformation(Info_S);

                this->graph.addEdge(s_edge);

            }
        }
    }

    void SFMgraph::AddImageEdges(JPL7 *Camera_vertex, cv::Mat Image, vector<Point> Image_Features,
                                 vector<Feature *> Feature_list) {
        //Add Image Edge

        for (size_t i = 0; i < Feature_list.size(); i++) {
            ImageEdge *i_edge = new ImageEdge();
            i_edge->setVertex(0, Camera_vertex);
            i_edge->setVertex(1, Feature_list[i]);


            Eigen::Matrix<double, 2, 1> Meas;
            Meas << Image_Features[i].x, Image_Features[i].y;
            i_edge->setMeasurement(Meas);

            //Just a placeholder for the true Info
            Eigen::Matrix<double, 2, 2> Info = Eigen::MatrixXd::Identity(2, 2);

            i_edge->setInformation(Info);

            this->graph.addEdge(i_edge);
        }
    }


    Eigen::Matrix<double,3,1> SFMgraph::triangulate_point (JPL7 *Camera_1, JPL7 *Camera_2, Point2d uv1, Point2d uv2) {

        Eigen::Matrix<double, 7, 1> est_1 = Camera_1->estimate();

        Eigen::Matrix<double, 7, 1> est_2 = Camera_2->estimate();

        Eigen::Matrix<double, 3, 3> R_G_to_1 = quat_2_Rot(est_1.block(0, 0, 4, 1));

        Eigen::Matrix<double, 3, 3> R_G_to_2 = quat_2_Rot(est_2.block(0, 0, 4, 1));

        Eigen::Matrix<double, 3, 3> R_2_to_1 = R_G_to_1 * R_G_to_2.transpose();

        Eigen::Matrix<double, 3, 1> p_2_in_G = est_2.block(4, 0, 3, 1);

        Eigen::Matrix<double, 3, 1> p_1_in_G = est_1.block(4, 0, 3, 1);

        Eigen::Matrix<double, 3, 1> p_2_in_1 = R_G_to_1 * (p_2_in_G - p_1_in_G);

        //Build Triangulation system equation


        Eigen::Matrix<double, 3, 1> b_1;
        b_1 << uv1.x, uv1.y, 1;

        Eigen::Matrix<double, 3, 1> b_2;
        b_2 << uv2.x, uv2.y, 1;


        Eigen::Matrix<double, 3, 1> b_2_in_1 = R_2_to_1 * b_2;

        Eigen::Matrix<double, 2, 2> A;

        A << b_1(0, 0), -b_2_in_1(0, 0),
                b_1(1, 0), -b_2_in_1(1, 0);

        Eigen::Matrix<double, 2, 1> rhs = p_2_in_1.block(0, 0, 2, 1);

        Eigen::Matrix<double, 2, 1> alpha_beta = A.inverse() * rhs;


        Eigen::Matrix<double, 3, 1> p_f_in_1 = alpha_beta(0, 0) * b_1;

        Eigen::Matrix<double, 3, 1> p_f_in_G = R_G_to_1.transpose() * p_f_in_1 + p_1_in_G;

        return p_f_in_G;
    }

    void SFMgraph::find_camera_from_features(JPL7 *Camera_1, vector<Eigen::Matrix<double, 3, 1>> uv,
                                             vector<Feature_Class *> Feature_Class_list) {
        int n = uv.size();

        vector<Feature*> Feature_list;
        for (int i = 0; i < Feature_Class_list.size(); i++) {
            Feature_list.push_back(Feature_Class_list[i]->feature);
        }
        //center of the sets
        Eigen::Matrix<double, 3, 1> pc_in_R;
        Eigen::Matrix<double, 3, 1> pc_in_G;

        for (size_t i = 0; i < n; i++) {
            pc_in_R += uv[i];
            pc_in_G += Feature_list[i]->estimate();
        }

        pc_in_R = pc_in_R / n;

        pc_in_G = pc_in_G / n;


        Eigen::Matrix<double, 3, 3> M;
        M.setZero();


        for (size_t i = 0; i < n; i++) {
            M += (uv[i] - pc_in_R) * (Feature_list[i]->estimate() - pc_in_G).transpose();
        }

        Eigen::Matrix<double, 3, 3> P = M.transpose() * M;

        //cout << "P - " << endl << P << endl;

        Eigen::Matrix<double, 3, 3> Pinv= P.inverse();

        Eigen::MatrixSquareRoot<Eigen::Matrix<double,3,3>,0> sqrm(Pinv);

        Eigen::Matrix<double, 3, 3> Psqrinv;

        sqrm.compute(Psqrinv);

        Eigen::Matrix<double, 3, 3> U = M * (Psqrinv);

        Eigen::Matrix<double, 3, 3> R = (U.determinant() * U).transpose();

        //cout << "R- " << endl << R << endl;

        Eigen::Matrix<double, 7, 1> pcam_in_G;




        pcam_in_G.block(0, 0, 4, 1) = rot_2_quat(R.transpose());

        pcam_in_G.block(4, 0, 3, 1) = pc_in_G - R * pc_in_R;

        cout << "pcam_in_G- " << endl << pcam_in_G<< endl;

        //cout << "pc_in_G- " << endl << pc_in_G << endl;

        Camera_1->setEstimate(pcam_in_G);

    }

    Eigen::Matrix<double,3,1> SFMgraph::find_feature_from_image_and_sonar(JPL7 *Camera, JPL7* Calib, Eigen::Matrix<double, 3, 1> uv, double r) {

    }
}
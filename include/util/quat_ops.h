#pragma once
#include <string>
#include <sstream>
#include <Eigen/Dense>

/**
Vertex based on
**/

namespace MAST
{

inline Eigen::Matrix<double,4,1> rot_2_quat(const Eigen::MatrixXd &rot)
{

    assert(rot.cols() == 3);
    assert(rot.rows() == 3);
    Eigen::Matrix<double,4,1> q;
    double T = rot.trace();
    if ((rot(0, 0) > T) && (rot(0, 0) > rot(1, 1)) && (rot(0, 0) > rot(2, 2)))
    {
        q(0) = sqrt((1 + (2 * rot(0, 0)) - T) / 4);
        q(1) = (1 / (4 * q(0))) * (rot(0, 1) + rot(1, 0));
        q(2) = (1 / (4 * q(0))) * (rot(0, 2) + rot(2, 0));
        q(3) = (1 / (4 * q(0))) * (rot(1, 2) - rot(2, 1));

    }
    else if ((rot(1, 1) > T) && (rot(1, 1) > rot(0, 0)) && (rot(1, 1) > rot(2, 2)))
    {
        q(1) = sqrt((1 + (2 * rot(1, 1)) - T) / 4);
        q(0) = (1 / (4 * q(1))) * (rot(0, 1) + rot(1, 0));
        q(2) = (1 / (4 * q(1))) * (rot(1, 2) + rot(2, 1));
        q(3) = (1 / (4 * q(1))) * (rot(2, 0) - rot(0, 2));
    }
    else if ((rot(2, 2) > T) && (rot(2, 2) > rot(0, 0)) && (rot(2, 2) > rot(1, 1)))
    {
        q(2) = sqrt((1 + (2 * rot(2, 2)) - T) / 4);
        q(0) = (1 / (4 * q(2))) * (rot(0, 2) + rot(2, 0));
        q(1) = (1 / (4 * q(2))) * (rot(1, 2) + rot(2, 1));
        q(3) = (1 / (4 * q(2))) * (rot(0, 1) - rot(1, 0));
    }
    else
    {

        q(3) = sqrt((1 + T) / 4);
        q(0) = (1 / (4 * q(3))) * (rot(1, 2) - rot(2, 1));
        q(1) = (1 / (4 * q(3))) * (rot(2, 0) - rot(0, 2));
        q(2) = (1 / (4 * q(3))) * (rot(0, 1) - rot(1, 0));
    }
    if (q(3) < 0)
    {
        q = -q;
    }
    q = q /(q.norm());
    return q;
}



inline Eigen::MatrixXd skew_x(Eigen::Matrix<double, 3,1> w)
{
    Eigen::Matrix<double, 3,3> w_x;
    w_x <<          0, -w(2), w(1),
        w(2), 0 , -w(0),
        -w(1), w(0), 0;
    return w_x;
}

inline Eigen::MatrixXd quat_2_Rot(Eigen::Matrix<double, 4,1> q)

{
    Eigen::Matrix<double,3,3> q_x= skew_x(q.block(0,0,3,1));
    Eigen::MatrixXd Rot = (2*pow(q(3,0),2)-1)*Eigen::MatrixXd::Identity(3,3)-2*q(3,0)*q_x+2*q.block(0,0,3,1)*(q.block(0,0,3,1).transpose());
    return Rot;
}


inline Eigen::Matrix<double,4,1>  quat_multiply(Eigen::Matrix<double,4,1> q, Eigen::Matrix<double,4,1> p )
{
    Eigen::Matrix<double,4,1> q_t;
    Eigen::Matrix<double,4,4> Qm;
    Qm.block(0,0,3,3) = q(3,0)*Eigen::MatrixXd::Identity(3,3)-skew_x(q.block(0,0,3,1));
    Qm.block(0,3,3,1) = q.block(0,0,3,1);
    Qm.block(3,0,1,3) = -q.block(0,0,3,1).transpose();
    Qm(3,3) = q(3,0);
    q_t= Qm*p;
    return q_t;

}
}

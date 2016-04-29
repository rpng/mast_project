#pragma once
//#include "util/SophusUtil.h"

#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_multi_edge.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "../util/quat_ops.h"
#include "g2o/core/base_unary_edge.h"


#include <string>
#include <sstream>
#include <Eigen/Dense>

/**
Vertex based on
**/

using namespace std;

namespace MAST
{




class JPL7 : public g2o::BaseVertex<6, Eigen::Matrix<double,7,1> >
{
public:

    /**
    state-       error-
    q            dtheta
    p            dp




    **/
    JPL7();
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    virtual void setToOriginImpl() {
    }

    virtual void oplusImpl(const double* update_)
    {
        Eigen::Matrix<double, 4,1 > q_minus = estimate().block(0,0,4,1);
        Eigen::Matrix<double, 3,1> rest_minus = estimate().block(4,0,3,1);
        Eigen::Matrix<double, 7, 1> Upd_vec;
        Eigen::Map< Eigen::Matrix<double, 6, 1> > update(const_cast<double*>(update_));
        Eigen::Matrix<double,3,1> d_theta = update.block(0,0,3,1);
        Eigen::Matrix<double, 3,1> rest_upd = update.block(3,0,3,1);
        Eigen::Matrix<double,4,1> d_q;
        Eigen::Matrix<double,3,1> d_qvec = .5*d_theta;
        d_q.block(0,0,3,1)= d_qvec;
        if(d_qvec.norm() > 1){
            d_q(4,0) = 1;
            d_q= (1/(sqrt(1+ d_qvec.transpose()*d_qvec)))*d_q;
        }
        else
        {
            d_q(4,0)= sqrt(1- d_qvec.transpose()*d_qvec);
        }

        Upd_vec.block(0,0,4,1) = quat_multiply(d_q,q_minus );
        Upd_vec.block(4,0,3,1) = rest_minus+rest_upd;



        setEstimate(Upd_vec);
    }
};





class Feature : public g2o::BaseVertex<3, Eigen::Matrix<double,3,1> >
{
public:


    Feature();
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    virtual void setToOriginImpl() {
    }

    virtual void oplusImpl(const double* update_)
    {

        Eigen::Map< Eigen::Matrix<double, 3, 1> > update(const_cast<double*>(update_));
        Eigen::Map< Eigen::Matrix<double, 3, 1> > est_up = estimate()+ update;

        setEstimate(est_up);
    }
};



/**
* \brief Bias edge between two JPL16's
*/


class ImageEdge : public g2o::BaseBinaryEdge<2, Eigen::Matrix<double, 2,1>, JPL7, Feature>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    BiasEdge();

    double delta_t;

    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    void computeError()
    {
        const JPL7* _cam = static_cast<const JPL16*>(_vertices[0]);
        const JPL16* _feat = static_cast<const JPL16*>(_vertices[1]);

        Eigen::Matrix<double,7,1> cam_est = _cam->estimate();
        Eigen::Matrix<double,3,1> feat_est = _feat->estimate();


        Eigen::Matrix<double,3,3> R = quat_2_Rot(cam_est.block(0,0,4,1));
        Eigen::Matrix<double,3,1> p_c = cam_est.block(4,0,3,1);

        Eigen::Matrix<double,3,1> c_p_f = R*(feat_est- p_c);

        double u = (c_p_f(0,0))/(c_p_f(2,0));

        double v = (c_p_f(1,0))/(c_p_f(2,0));


        Eigen::Matrix<double,2,1> z_hat;

        z_hat << u, v;

        _error= z_hat- _measurement;


         //cout << "Biasinf" << endl << _information << endl;

    }

    void linearizeOplus()
    {
        const JPL7* _cam = static_cast<const JPL16*>(_vertices[0]);
        const JPL16* _feat = static_cast<const JPL16*>(_vertices[1]);

        Eigen::Matrix<double,7,1> cam_est = _cam->estimate();
        Eigen::Matrix<double,3,1> feat_est = _feat->estimate();


        Eigen::Matrix<double,3,3> R = quat_2_Rot(cam_est.block(0,0,4,1));
        Eigen::Matrix<double,3,1> p_c = cam_est.block(4,0,3,1);

        Eigen::Matrix<double,3,1> c_p_f = R*(feat_est- p_c);

        double x = (c_p_f(1,0));

        double y = (c_p_f(1,0));

        double z = (c_p_f(2,0));



        Eigen::Matrix<double,2,3> Hpf;

        Hpf << (1/z), 0, -(x/(pow(z,2))),
                 0, (1/z),-(y/(pow(z,2)));


        Eigen::Matrix<double,3,9> H_i;

        H_i.block(0,0,3,3) = skew_x(R*(feat_est-p_c));

        H_i.block(0,3,3,3) = -R;

        H_i.block(0,6,3,3) = R;


        Eigen::Matrix<double,2,9> H = Hpf*H_i;





        _jacobianOplusXj = H.block(0,0,2,6);
        _jacobianOplusXi = H.block(0,6,2,3);

        /*cout << "H_j" << endl << H_j << endl << endl;

            cout << "H_i" << endl << H_i << endl << endl;

            std::exit(1);*/


    }



};


}

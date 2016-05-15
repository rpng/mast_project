#pragma once
//#include "util/SophusUtil.h"

#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_multi_edge.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "util/quat_ops.h"
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
            d_q(3,0) = 1;
            d_q= (1/(sqrt(1+ d_qvec.transpose()*d_qvec)))*d_q;
        }
        else
        {
            d_q(3,0)= sqrt(1- d_qvec.transpose()*d_qvec);
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
        Eigen::Matrix<double, 3, 1>  est_up = estimate()+ update.block(0,0,3,1);

        setEstimate(est_up);
    }

};



class ImageEdge : public g2o::BaseBinaryEdge<2, Eigen::Matrix<double, 2,1>, JPL7, Feature>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ImageEdge();

    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    void computeError()
    {
        const JPL7* _cam = static_cast<const JPL7*>(_vertices[0]);
        const Feature* _feat = static_cast<const Feature*>(_vertices[1]);

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
        const JPL7* _cam = static_cast<const JPL7*>(_vertices[0]);
        const Feature* _feat = static_cast<const Feature*>(_vertices[1]);

        Eigen::Matrix<double,7,1> cam_est = _cam->estimate();
        Eigen::Matrix<double,3,1> feat_est = _feat->estimate();


        Eigen::Matrix<double,3,3> R = quat_2_Rot(cam_est.block(0,0,4,1));
        Eigen::Matrix<double,3,1> p_c = cam_est.block(4,0,3,1);

        Eigen::Matrix<double,3,1> c_p_f = R*(feat_est- p_c);

        double x = (c_p_f(0,0));

        double y = (c_p_f(1,0));

        double z = (c_p_f(2,0));



        Eigen::Matrix<double,2,3> Hpf;

        Hpf << (1/z), 0, -(x/(pow(z,2))),
                 0, (1/z),-(y/(pow(z,2)));


        Eigen::Matrix<double,3,9> H_i;

        H_i.block(0,0,3,3) = skew_x(R*(feat_est-p_c));

        H_i.block(0,3,3,3) = -R;

        H_i.block(0,6,3,3) = R;

        /*cout << "H_i" << endl << H_i << endl;


        cout << "H_pf" << endl << Hpf << endl;

        std::exit(1);*/



        Eigen::Matrix<double,2,9> H = Hpf*H_i;





        _jacobianOplusXi = H.block(0,0,2,6);
        _jacobianOplusXj = H.block(0,6,2,3);

        /*cout << "H_j" << endl << H_j << endl << endl;

            cout << "H_i" << endl << H_i << endl << endl;

            std::exit(1);*/


    }



};


class SonarEdge: public g2o::BaseMultiEdge<2,Eigen::Matrix<double,2,1> >
{



public:

    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    SonarEdge();



    void setToOriginImpl()
    {
    }


    void computeError()
    {

         const JPL7* _calib = static_cast<const JPL7*>(_vertices[0]);
        const JPL7* _cam = static_cast<const JPL7*>(_vertices[1]);
        const Feature* _feat = static_cast<const Feature*>(_vertices[2]);

        Eigen::Matrix<double, 7,1 > calib= _calib->estimate();

        Eigen::Matrix<double, 7,1 > cam= _cam->estimate();

        Eigen::Matrix<double, 3,1> feat= _feat->estimate();

        Eigen::Matrix<double, 3,3> R_G_to_C= quat_2_Rot(cam.block(0,0,4,1));

        Eigen::Matrix<double, 3,1> p_c_in_g= cam.block(4,0,3,1);

        Eigen::Matrix<double, 3,3> R_c_to_s = quat_2_Rot(calib.block(0,0,4,1));

        Eigen::Matrix<double, 3,1> p_c_in_s = calib.block(4,0,3,1);

        Eigen::Matrix<double,3,1> p_f_in_s= R_c_to_s*R_G_to_C*(feat- p_c_in_g)+ p_c_in_s;

        double r_hat= p_f_in_s.norm();

        double th_hat= atan2(p_f_in_s(1,0), p_f_in_s(0,0));

        Eigen::Matrix<double, 2,1> z_hat;

        z_hat(0,0) = r_hat;

        z_hat(1,0) = th_hat;

        _error = z_hat-_measurement;





    }

    void linearizeOplus()
    {

        const JPL7* _calib = static_cast<const JPL7*>(_vertices[0]);
        const JPL7* _cam = static_cast<const JPL7*>(_vertices[1]);
        const Feature* _feat = static_cast<const Feature*>(_vertices[2]);

        Eigen::Matrix<double, 7,1 > calib= _calib->estimate();

        Eigen::Matrix<double, 7,1 > cam= _cam->estimate();

        Eigen::Matrix<double, 3,1> feat= _feat->estimate();

        Eigen::Matrix<double, 3,3> R_G_to_C= quat_2_Rot(cam.block(0,0,4,1));

        Eigen::Matrix<double, 3,1> p_c_in_g= cam.block(4,0,3,1);

        Eigen::Matrix<double, 3,3> R_c_to_s = quat_2_Rot(calib.block(0,0,4,1));

        Eigen::Matrix<double, 3,1> p_c_in_s = calib.block(4,0,3,1);

        Eigen::Matrix<double,3,1> p_f_in_s= R_c_to_s*R_G_to_C*(feat- p_c_in_g)+ p_c_in_s;

        double x= p_f_in_s(0,0);

        double y= p_f_in_s(1,0);

        double z= p_f_in_s(2,0);

        double p_norm= p_f_in_s.norm();


        Eigen::Matrix<double, 2,3> H_spc;
        H_spc.setZero();

        H_spc(0,0)= (x/p_norm);

        H_spc(0,1) = (y/p_norm);

        H_spc(0,2) = (z/p_norm);

        H_spc(1,0)= (-y/pow(x,2))/(1+pow((y/x),2));

        H_spc(1,1) = (1/x)/(1+pow((y/x),2));


        //Derivative with respect to cam

        Eigen::Matrix<double, 3,6> H_c;

        H_c.setZero();

        H_c.block(0,0,3,3)= R_c_to_s*skew_x(R_G_to_C*(feat-p_c_in_g));

        H_c.block(0,3,3,3) = -R_c_to_s*R_G_to_C;

        //Derivative with respect to feat


        Eigen::Matrix<double, 3,3> H_f= R_c_to_s*R_G_to_C;

        //Derivative with respect to calib


        Eigen::Matrix<double, 3,6> H_ca;

        H_ca.setZero();

        H_ca.block(0,0,3,3)= skew_x(R_c_to_s*R_G_to_C*(feat-p_c_in_g));

        H_ca.block(0,3,3,3)= Eigen::MatrixXd::Identity(3,3);


        /*cout << "H_ca- " << endl << H_ca << endl;

        std::exit(1);*/



        _jacobianOplus[0]= H_spc*H_ca;
        _jacobianOplus[1]= H_spc*H_c;
        _jacobianOplus[2]= H_spc*H_f;





    }



};

    class PriorEdge : public g2o::BaseUnaryEdge<6, Eigen::Matrix<double,6,1>, JPL7 >
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        PriorEdge();

        bool read(std::istream& is);
        bool write(std::ostream& os) const;

        Eigen::Matrix<double,7,1> prior_meas;

        void computeError()
        {
            //cout << "Prior error" << endl;
            const JPL7* prior = static_cast<const JPL7*>(_vertices[0]);

            Eigen::Matrix<double,7,1> p_est= prior->estimate();

            Eigen::Matrix<double,4,1> est_quat= p_est.block(0,0,4,1);

            Eigen::Matrix<double,4,1> meas_quat= prior_meas.block(0,0,4,1);

            Eigen::Matrix<double,4,1> err_quat= rot_2_quat(quat_2_Rot(est_quat)*(quat_2_Rot(meas_quat).transpose()));


            Eigen::Matrix<double,6,1> error_;

            error_.block(0,0,3,1) = 2*err_quat.block(0,0,3,1);
            error_.block(3,0,3,1) = p_est.block(4,0,3,1)- prior_meas.block(4,0,3,1);



            _error= error_;

            //cout << "Prior error end" << endl;
        }

        void linearizeOplus()
        {
            const JPL7* prior = static_cast<const JPL7*>(_vertices[0]);

            Eigen::Matrix<double,7,1> p_est= prior->estimate();

            Eigen::Matrix<double,4,1> est_quat= p_est.block(0,0,4,1);

            Eigen::Matrix<double,4,1> meas_quat= prior_meas.block(0,0,4,1);

            Eigen::Matrix<double,4,1> err_quat= rot_2_quat(quat_2_Rot(est_quat)*quat_2_Rot(meas_quat).transpose());

            Eigen::Matrix<double,6,6> Hi;

            Hi.setZero();

            Hi.block(0,0,3,3)=  err_quat(3,0)*Eigen::MatrixXd::Identity(3,3)+ skew_x(err_quat.block(0,0,3,1));

                    //Eigen::MatrixXd::Identity(3,3);

            Hi.block(3,3,3,3)= Eigen::MatrixXd::Identity(3,3);


            /*cout << "pri Jac" << endl << Hi << endl;

            std::exit(1);*/




            _jacobianOplusXi = Hi;
            //cout << "Lin pri begin" << endl;
        }



    };



}

#pragma once
//#include "util/SophusUtil.h"

#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_multi_edge.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/core/robust_kernel_impl.h"
#include "VertexGravity.h"
#include "../util/quat_ops.h"
#include "opencv2/opencv.hpp"
#include "g2o/core/base_unary_edge.h"


#include <string>
#include <sstream>
#include <Eigen/Dense>

/**
Vertex based on
**/

using namespace std;

namespace Pre_int
{




class JPL16 : public g2o::BaseVertex<15, Eigen::Matrix<double,16,1> >
{
public:

    /**
    state-       error-
    q            dtheta
    bw           dbw
    v            dv
    ba           dba
    p            dp




    **/
    JPL16();
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    virtual void setToOriginImpl() {
    	_estimate.setZero();
    	_estimate(3,0) =   1.0;
    }

    virtual void oplusImpl(const double* update_)
    {
        Eigen::Matrix<double, 4,1 > q_minus = estimate().block(0,0,4,1);
        Eigen::Matrix<double, 12,1> rest_minus = estimate().block(4,0,12,1);
        Eigen::Matrix<double, 16, 1> Upd_vec;
        Eigen::Map< Eigen::Matrix<double, 15, 1> > update(const_cast<double*>(update_));
        Eigen::Matrix<double,3,1> d_theta = update.block(0,0,3,1);
        Eigen::Matrix<double, 12,1> rest_upd = update.block(3,0,12,1);
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
        Upd_vec.block(4,0,12,1) = rest_minus+rest_upd;



        setEstimate(Upd_vec);
    }
};

/*
class MargEdge : public g2o::BaseBinaryEdge<33, Eigen::Matrix<double,33,1>, JPL16, JPL16 >
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	MargEdge();


    void setToOriginImpl()
    {
    }



	Eigen::Matrix<double, 35,1> est;
	Eigen::Matrix<double, 4,1> q_1_bar_inv;
	Eigen::Matrix<double, 4,1> q_2_bar_inv;



	bool read(std::istream& is);
	bool write(std::ostream& os) const;

	void computeError()
	{

	const VertexGrav* _grav =  static_cast<const VertexGrav*>(_vertices[0]);

    const JPL16* node_1 = static_cast<const JPL16*>(_vertices[1]);

    const JPL16* node_2 = static_cast<const JPL16*>(_vertices[2]);

    Eigen::Matrix<double,3,1> g_est= _grav->estimate();

    Eigen::Matrix<double,16,1> node_1_est=  node_1->estimate();

    Eigen::Matrix<double,16,1> node_2_est= node_2->estimate();

    Eigen::Matrix<double,4,1> res_q_1 = quat_multiply(node_1_est.block(3,0,4,1),q_1_bar_inv);

    Eigen::Matrix<double,4,1> res_q_2 = quat_multiply(node_2_est.block(19,0,4,1),q_2_bar_inv);

    Eigen::Matrix<double, 33,1> error_;

    error_.setZero();

    //Computing Error

    error_.block(0,0,3,1) = g_est- est.block(0,0,3,1);

    error_.block(3,0,3,1) = 2*res_q_1.block(0,0,3,1);

    error_.block(6,0,12,1) = node_1_est.block(4,0,12,1)- est(7,0,12,1);

    error_block(18,0,3,1) = 2*res_q_1.block(0,0,3,1);

    error_.block(21,0,12,1) = node_2_est.block(4,0,12,1)- est(23,0,12,1);








	}

	void linearizeOplus()
	{
		//const VertexGrav* g_vert = static_cast<const VertexGrav*>(_vertices[0]);
		//Eigen::Matrix<double, 3,1> g_ = g_vert->estimate();
		//Eigen::MatrixXd gT= g_.transpose();

			const VertexGrav* _grav =  static_cast<const VertexGrav*>(_vertices[0]);

            const JPL16* node_1 = static_cast<const JPL16*>(_vertices[1]);

            const JPL16* node_2 = static_cast<const JPL16*>(_vertices[2]);

            Eigen::Matrix<double,16,1> g_est= _grav->estimate();

            Eigen::Matrix<double,16,1> node_1_est=  node_1->estimate();

            Eigen::Matrix<double,16,1> node_2_est= node_2->estimate();

            Eigen::Matrix<double,4,1> res_q_1 = quat_multiply(node_1_est.block(0,0,4,1),q_1_bar_inv);

            Eigen::Matrix<double,4,1> res_q_2 = quat_multiply(node_2_est.block(0,0,4,1),q_2_bar_inv);

            Eigen::Matrix<double,33,3> H_g;

            Eigen::Matrix<double,33,15> H_i;

            Eigen::Matrix<double,33,15> H_j;

            H_g.setZero();

            H_i.setZero();

            H_j.setZero();

            H



	}



};*/





class PriorEdge : public g2o::BaseUnaryEdge<6, Eigen::Matrix<double,6,1>, JPL16 >
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	PriorEdge();

	bool read(std::istream& is);
	bool write(std::ostream& os) const;

	void computeError()
	{
    const JPL16* prior = static_cast<const JPL16*>(_vertices[0]);

    Eigen::Matrix<double,16,1> p_est= prior->estimate();

    Eigen::Matrix<double,6,1> error_;

    error_.block(0,0,3,1) = 2*p_est.block(0,0,3,1);
    error_.block(3,0,3,1) = p_est.block(13,0,3,1);

    _error= error_;
	}

	void linearizeOplus()
	{
		//const VertexGrav* g_vert = static_cast<const VertexGrav*>(_vertices[0]);
		//Eigen::Matrix<double, 3,1> g_ = g_vert->estimate();
		//Eigen::MatrixXd gT= g_.transpose();

		Eigen::Matrix<double, 6, 15> HI;
		HI.setZero();

		HI.block(0,0,3,3) = 2*Eigen::MatrixXd::Identity(3,3);

		HI.block(3,13,3,3) = Eigen::MatrixXd::Identity(3,3);


		_jacobianOplusXi = Eigen::MatrixXd::Identity(3,3);
	}



};

/**
* \brief Bias edge between two JPL16's
*/


class BiasEdge : public g2o::BaseBinaryEdge<6, Eigen::Matrix<double, 6,1>, JPL16, JPL16>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    BiasEdge();

    double delta_t;

    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    void computeError()
    {
        const JPL16* _from = static_cast<const JPL16*>(_vertices[0]);
        const JPL16* _to = static_cast<const JPL16*>(_vertices[1]);

        Eigen::Matrix<double,6,1> error_;
        error_.block(0,0,3,1)= _to->estimate().block(4,0,3,1) -_from->estimate().block(4,0,3,1);
        error_.block(3,0,3,1)= _to->estimate().block(10,0,3,1) -_from->estimate().block(10,0,3,1);
        _error= error_;

         if (std::isnan(error_.norm())){
            cout << "Error in Bias" << endl;

        cout << "Bias Check" << endl;
         cout << "to_est_bw" << endl << _to->estimate().block(4,0,3,1) << endl;
          cout << "from_est_bw"<< endl << _from->estimate().block(4,0,3,1) << endl;
        cout << "to_est_ba" << endl <<_to->estimate().block(10,0,3,1) << endl;
         cout << "from_est_ba" << endl <<_from->estimate().block(10,0,3,1) << endl;
         std::exit(1);}

         //cout << "Biasinf" << endl << _information << endl;

    }

    void linearizeOplus()
    {

        Eigen::Matrix<double, 6,15> H_i;
        Eigen::Matrix<double, 6,15> H_j;

        H_i.setZero();

        H_j.setZero();

        H_j.block(0,3,3,3)= Eigen::MatrixXd::Identity(3,3);
        H_i.block(0,3,3,3)= -Eigen::MatrixXd::Identity(3,3);

        H_j.block(3,9,3,3)= Eigen::MatrixXd::Identity(3,3);
        H_i.block(3,9,3,3)= -Eigen::MatrixXd::Identity(3,3);





        _jacobianOplusXj = H_j;
        _jacobianOplusXi = H_i;

        /*cout << "H_j" << endl << H_j << endl << endl;

            cout << "H_i" << endl << H_i << endl << endl;

            std::exit(1);*/

        if (std::isnan(H_j.norm()) || std::isnan(H_i.norm())){
            cout << "Error in Bias JAC" << endl;
            cout << "H_j" << endl << H_j << endl << endl;

            cout << "H_i" << endl << H_i << endl << endl;
            std::exit(1);
            }

    }



};

class imuEdge: public g2o::BaseMultiEdge<9,Eigen::Matrix<double,9,1> >
{



public:

    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    imuEdge();

    Eigen::Matrix<double, 9,15> H_i;

    Eigen::Matrix<double, 9,15> H_j;

    Eigen::Matrix<double, 9,3> H_g;

    Eigen::Matrix<double, 3,1> b_w_minus;

    Eigen::Matrix<double, 3,1> b_a_minus;

    Eigen::Matrix<double, 3,1> alpha;
    Eigen::Matrix<double, 3,1> beta;
    Eigen::Matrix<double, 3,3> Rot;
    double delta_t;

    Eigen::Matrix<double, 3,6> _J_alpha;
    Eigen::Matrix<double, 3,6> _J_beta;
    Eigen::Matrix<double, 3,6> _J_q;

    void setToOriginImpl()
    {
    }

    void set_bias_jacobs(Eigen::Matrix<double, 3,6> J_alpha, Eigen::Matrix<double, 3,6> J_beta, Eigen::Matrix<double, 3,6> J_q)
    {
        _J_alpha= J_alpha;
        _J_beta= J_beta;
        _J_q= J_q;
    }

    void computeError()
    {

        const VertexGrav* _grav = static_cast<const VertexGrav*>(_vertices[0]);
        const JPL16* _from = static_cast<const JPL16*>(_vertices[1]);
        const JPL16* _to = static_cast<const JPL16*>(_vertices[2]);
        Eigen::Matrix<double, 16,1> from_est= _from->estimate();
        Eigen::Matrix<double, 16,1> to_est = _to->estimate();

        //cout << "From_est" << endl << from_est << endl << endl;

        //cout << "to_est" << endl << to_est << endl << endl;

        Eigen::Matrix<double, 3,1> g= _grav->estimate();

       // cout << "qk" << endl << from_est.block(0,0,4,1)<< endl << endl << "Rk test" << endl << quat_2_Rot(from_est.block(0,0,4,1))<< endl << endl;

        Eigen::Matrix<double, 3,3> Rk= quat_2_Rot(from_est.block(0,0,4,1));

        //cout << "Rk" << endl << Rk << endl << endl;

        Eigen::Matrix<double, 3,3> Rk1= quat_2_Rot(to_est.block(0,0,4,1));

        //cout << "Rk1" << endl << Rk1 << endl << endl;

        Eigen::Matrix<double, 3,1> b_wk= from_est.block(4,0,3,1);

        //cout << "b_wk" << endl << b_wk << endl << endl;

        Eigen::Matrix<double, 3,1> vk= from_est.block(7,0,3,1);

        //cout << "v_k" << endl << vk << endl << endl;

        Eigen::Matrix<double, 3,1> vk1= to_est.block(7,0,3,1);

        //cout << "vk1" << endl << vk1 << endl << endl;

        Eigen::Matrix<double, 3,1> b_ak= from_est.block(10,0,3,1);

       // cout << "b_ak" << endl << b_ak << endl << endl;

        Eigen::Matrix<double, 3,1> pk= from_est.block(13,0,3,1);

        //cout << "pk" << endl << pk << endl << endl;

        Eigen::Matrix<double, 3,1> pk1= to_est.block(13,0,3,1);

        //cout << "pk1" << endl << pk1 << endl << endl;

        Eigen::Matrix<double, 9,1> error_;

        Eigen::Matrix<double, 6,1> db;
        db.block(0,0,3,1) = b_wk- b_w_minus;
        db.block(3,0,3,1) = b_ak- b_a_minus;


        error_.block(0,0,3,1)= Rk*(pk1-pk-vk*delta_t+.5*g*delta_t*delta_t) - _J_alpha*db- alpha;
        error_.block(3,0,3,1)= Rk*(vk1-vk+g*delta_t) - _J_beta*db-beta;


        /*cout << "Rk" << endl << Rk << endl << endl;

        cout << "g " << endl << g << endl << endl;

        cout << "eb-2" << endl << (pk1-pk-vk*delta_t+.5*g*delta_t*delta_t) << endl << endl;

        cout << "eb-1" << endl << vk1-vk+g*delta_t << endl << endl;

        cout << "eb0" << endl << Rk*(pk1-pk-vk*delta_t+.5*g*delta_t*delta_t) << endl;

        cout << "eb1" << endl << _J_alpha*db << endl << endl;

        cout << "eb2" << endl << Rk*(vk1-vk+g*delta_t) << endl << endl;

        cout << "eb3" << endl << _J_beta*db << endl << endl;

        cout << "eb4" << endl << Rk*(pk1-pk-vk*delta_t+.5*g*delta_t*delta_t) - _J_alpha*db << endl << endl;


        cout << "eb5" << endl << Rk*(vk1-vk+g*delta_t) - _J_beta*db << endl << endl;*/

        Eigen::Matrix<double, 3,3> Rr;
        Rr = (Eigen::MatrixXd::Identity(3,3)- skew_x(_J_q*db))*Rk1*Rk.transpose()*Rot.transpose();

        /*cout << "Rr" << endl << Rr << endl << endl;


        cout << "dqe-" << endl;
        cout << rot_2_quat(Rr) << endl << endl;*/


        Eigen::Matrix<double, 4,1> dqr = rot_2_quat(Rr);
        error_.block(6,0,3,1) = 2*dqr.block(0,0,3,1);
        _error= error_;

        if (std::isnan(error_.norm())){
            cout << "Error in IMU" << endl;
            std::exit(1);}
        /*cout << "IMUERROR-" << endl;
        cout << _error << endl;*/


    }

    void linearizeOplus()
    {

        const VertexGrav* _grav = static_cast<const VertexGrav*>(_vertices[0]);
        const JPL16* _from = static_cast<const JPL16*>(_vertices[1]);
        const JPL16* _to = static_cast<const JPL16*>(_vertices[2]);
        Eigen::Matrix<double, 16,1> from_est= _from->estimate();
        Eigen::Matrix<double, 16,1> to_est = _to->estimate();

        Eigen::Matrix<double, 3,1> g= _grav->estimate();

        Eigen::Matrix<double, 3,3> Rk= quat_2_Rot(from_est.block(0,0,4,1));
        Eigen::Matrix<double, 3,3> Rk1= quat_2_Rot(to_est.block(0,0,4,1));

        Eigen::Matrix<double, 3,1> vk= from_est.block(7,0,3,1);
        Eigen::Matrix<double, 3,1> vk1= to_est.block(7,0,3,1);

        Eigen::Matrix<double, 3,1> pk= from_est.block(13,0,3,1);
        Eigen::Matrix<double, 3,1> pk1= to_est.block(13,0,3,1);

        H_i.setZero();

        H_j.setZero();

        H_g.setZero();




        H_i.block(0,0,3,3)= skew_x(Rk*(pk1-pk-vk*delta_t+.5*g*delta_t*delta_t));
        H_i.block(0,3,3,3)= -_J_alpha.block(0,0,3,3);
        H_i.block(0,6,3,3) = -delta_t*Rk;
        H_i.block(0,9,3,3)= -_J_alpha.block(0,3,3,3);
        H_i.block(0,12,3,3)= -Rk;

        H_j.block(0,12,3,3) = Rk;

        H_g.block(0,0,3,3)= .5*delta_t*delta_t*Rk;




        H_i.block(3,0,3,3)= skew_x(Rk*(vk1-vk+g*delta_t));
        H_i.block(3,3,3,3)= -_J_beta.block(0,0,3,3);
        H_i.block(3,6,3,3) = -Rk;
        H_i.block(3,9,3,3)= -_J_alpha.block(0,3,3,3);

        H_j.block(3,6,3,3)= Rk;

        H_g.block(3,0,3,3) = delta_t*Rk;



        H_i.block(6,0,3,3) = -Rk1*Rk.transpose();
        H_i.block(6,3,3,3) = _J_q.block(0,0,3,3);

        H_j.block(6,0,3,3) = Eigen::MatrixXd::Identity(3,3);


        _jacobianOplus[0]= H_g;
        _jacobianOplus[1]= H_i;
        _jacobianOplus[2]= H_j;


        if (std::isnan(H_g.norm()) || std::isnan(H_i.norm()) || std::isnan(H_j.norm())  ){
            cout << "Error in IMU Jac" << endl;
            cout << "H_g- " << endl << H_g << endl<< endl;
            cout << "H_i- " << endl << H_i << endl << endl;
            cout << "H_j- " << endl << H_j << endl << endl;
            std::exit(1);}



    }



};




class DenseAlignEdge : public g2o::BaseBinaryEdge<1, Eigen::Matrix<double, 1,1>, JPL16, JPL16>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    DenseAlignEdge();

      virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;
    //g2o::RobustKernelHuber* rk;

    /*g2o::BaseBinaryEdge<1, Eigen::Matrix<double, 1,1>, JPL16, JPL16> ()
    {
        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        setRobustKernel(rk);
    }*/

    //Vector of pixel locations in 1's frame
    vector< Eigen::Matrix<double,3,1> > depth_map;
    vector< vector<int> > uv_map;

    //Image
    cv::Mat* I_1;
    cv::Mat* I_2;


    Eigen::Matrix<double,3,3> K;

    double i_depth;

    double i_var;

    //X and Y Gradient Images of 2



    Eigen::Matrix<double, 3,1> c_p_I;

    Eigen::Matrix<double, 3,3> c_R_I;


    double sigma;

    double sigma_i;

    Eigen::Matrix<double,3,1> _c1_p_f_i;

    double u_1;
    double v_1;

    double u_hom;
    double v_hom;




    double delta_t;


    void computeError()
    {
        const JPL16* _from = static_cast<const JPL16*>(_vertices[0]);
        const JPL16* _to = static_cast<const JPL16*>(_vertices[1]);

        Eigen::Matrix<double,16,1> Est_1= _from->estimate();


        Eigen::Matrix<double,16,1> Est_2= _to->estimate();

        /*cout << Est_1 << endl;
        cout << Est_2 << endl;*/


        Eigen::Matrix<double,4,1> q_1 = Est_1.block(0,0,4,1);
        Eigen::Matrix<double,4,1> q_2 = Est_2.block(0,0,4,1);

        Eigen::Matrix<double,3,1> p_1 = Est_1.block(13,0,3,1);
        Eigen::Matrix<double,3,1> p_2 = Est_2.block(13,0,3,1);

        Eigen::Matrix<double,3,3> N_R_1 = quat_2_Rot(q_1);
        Eigen::Matrix<double,3,3> N_R_2 = quat_2_Rot(q_2);
        Eigen::Matrix<double,3,3> _1_R_2 = N_R_2*N_R_1.transpose();


        Eigen::Matrix<double,1,2> HI;

        Eigen::Matrix<double,3,1> _c2_p_f_i = c_R_I*_1_R_2*c_R_I.transpose()*(_c1_p_f_i- c_p_I)-  c_R_I*N_R_2*(p_2-p_1)+ c_p_I;

        Eigen::Matrix<double,3,1> _c2_p_f_i_p = (1/_c2_p_f_i[2])*_c2_p_f_i;


        /*cout << "c_R_I" << endl << c_R_I << endl << endl;

        cout << "c_p_I" << endl << c_p_I << endl << endl;

        cout << "_c1_p_f_i" << endl << _c1_p_f_i << endl << endl;

        //std::exit(1);


        cout << "c_R_I*_1_R_2*c_R_I.transpose()*(_c1_p_f_i- c_p_I)" << endl << c_R_I*_1_R_2*c_R_I.transpose()*(_c1_p_f_i- c_p_I) << endl << endl;

        cout << "c_R_I*N_R_2*(p_2-p_1)+ c_p_I" << c_R_I*N_R_2*(p_2-p_1)+ c_p_I<< endl << endl;

        cout << "_c2_p_f_i" << endl << _c2_p_f_i << endl << endl;

        cout << "_c2_p_f_i_p" << endl << _c2_p_f_i_p << endl << endl;

        cout << "K" << endl << K << endl << endl;*/

        _c2_p_f_i_p= K*_c2_p_f_i_p;

        int u_2 =  _c2_p_f_i_p[0];
        int v_2 =  _c2_p_f_i_p[1];

        double r_i;
        if (!(u_2 < I_2->cols) || !(v_2 < I_2->rows)|| u_2 <0 || v_2 <0)
        {
            /*cout << u_2<< endl;
            cout << v_2 << endl;
            cout << I_2->cols << endl;
            cout << I_2->rows << endl;*/


            r_i = 0;
            /*cout << "Bad MATCH YO " << endl;
            cout << r_i << endl;
            std::exit(1);*/

        }
        else
        {
            /*cout << "GOOD MATCH YO " << endl;

            cout << v_2 << ", " << u_2 << endl;

            cout << v_hom << ", " << u_hom << endl;

            cout << (double) I_2->at<uchar>(v_2,u_2) << endl;

            cout << "here" << endl;

            cout << ((double) I_1->at<uchar>(v_hom,u_hom)) << endl;

            cout << "here 2" << endl;*/

            r_i = ((double) I_2->at<uchar>(v_2,u_2))- ((double) I_1->at<uchar>(v_hom,u_hom));

            //cout << r_i << endl;
            HI <<   (((double) I_2->at<uchar>(v_2,u_2+1))-((double) I_2->at<uchar>(v_2,u_2-1)))/2, (((double) I_2->at<uchar>(v_2-1,u_2))-((double) I_2->at<uchar>(v_2+1,u_2)))/2;



        }

        HI(0,0)= K(0,0)*HI(0,0);

        HI(0,1)= K(1,1)*HI(0,1);

        Eigen::Matrix<double,2,3> H_pf;
        H_pf.setZero();
        H_pf.block(0,0,1,1)<< 1/(_c2_p_f_i[2]);
        H_pf.block(0,2,1,1)<< (-_c2_p_f_i[0])/(_c2_p_f_i[2]);
        H_pf.block(1,1,1,1)<< 1/(_c2_p_f_i[2]);
        H_pf.block(1,2,1,1)<< (-_c2_p_f_i[1])/(_c2_p_f_i[2]);

        //cout << i_depth << endl;

        Eigen::Matrix<double,3,1> dp2_did = -c_R_I*_1_R_2*c_R_I.transpose()*(_c1_p_f_i)*(1/(i_depth));




        Eigen::Matrix<double,1,1> error_;

        double drp_div= HI*H_pf*dp2_did;

        //cout << "dp2_did" << dp2_did<< endl;

        //cout << "drp_div" << drp_div << endl;

        sigma= 2*pow(sigma_i,2)+pow(drp_div,2)*i_var;


        setInformation((1/sigma)*Eigen::MatrixXd::Identity(1,1));

         /*cout << "IMUinf" << endl << _information << endl;
         cout << r_i << endl;
         std::exit(1);*/

        error_(0,0) = r_i;


        if (std::isnan(r_i)){
            cout << "Error in dense" << endl;
            std::exit(1);}

        _error= error_;


        //std::exit(1);



    }





    void linearizeOplus()
    {
        const JPL16* _from = static_cast<const JPL16*>(_vertices[0]);
        const JPL16* _to = static_cast<const JPL16*>(_vertices[1]);

        Eigen::Matrix<double,16,1> Est_1= _from->estimate();
        Eigen::Matrix<double,16,1> Est_2= _to->estimate();


        Eigen::Matrix<double,4,1> q_1 = Est_1.block(0,0,4,1);
        Eigen::Matrix<double,4,1> q_2 = Est_2.block(0,0,4,1);

        Eigen::Matrix<double,3,1> p_1 = Est_1.block(13,0,3,1);
        Eigen::Matrix<double,3,1> p_2 = Est_2.block(13,0,3,1);

        Eigen::Matrix<double,3,3> N_R_1 = quat_2_Rot(q_1);
        Eigen::Matrix<double,3,3> N_R_2 = quat_2_Rot(q_2);
        Eigen::Matrix<double,3,3> _1_R_2 = N_R_2*N_R_1.transpose();


        Eigen::Matrix<double,3,1> _c2_p_f_i = c_R_I*_1_R_2*c_R_I.transpose()*(_c1_p_f_i- c_p_I)-  c_R_I*N_R_2*(p_2-p_1)+ c_p_I;

        Eigen::Matrix<double,3,1> _1_p_f_i = c_R_I.transpose()*(_c1_p_f_i- c_p_I);

        Eigen::Matrix<double,3,1> _2_p_f_i = c_R_I.transpose()*(_c2_p_f_i- c_p_I);


        Eigen::Matrix<double,3,1> _c2_p_f_i_p = (1/_c2_p_f_i[2])*_c2_p_f_i;

        _c2_p_f_i_p= K*_c2_p_f_i_p;

        int u_2 =  _c2_p_f_i_p[0];
        int v_2 =  _c2_p_f_i_p[1];




        Eigen::Matrix<double,1,2> HI;
        if (!(u_2 < I_2->cols) || !(v_2 < I_2->rows)|| u_2 <0 || v_2 <0)
        {
            //cout << "Bad Error" << endl;
            HI.block(0,0,1,1)<< 0;
            HI.block(0,1,1,1)<< 0;
        }
        else
        {
           // cout << "GOOD ERROR" << endl;

            HI <<   (((double)I_2->at<uchar>(v_2,u_2+1))-((double) I_2->at<uchar>(v_2,u_2-1)))/2, (((double) I_2->at<uchar>(v_2-1,u_2))-((double) I_2->at<uchar>(v_2+1,u_2)))/2;

        }

        HI(0,0)= K(0,0)*HI(0,0);

        HI(0,1)= K(1,1)*HI(0,1);

        Eigen::Matrix<double,2,3> H_pf;

        H_pf.setZero();
        H_pf.block(0,0,1,1)<< 1/(_c2_p_f_i[2]);
        H_pf.block(0,2,1,1)<< (-_c2_p_f_i[0])/(_c2_p_f_i[2]);
        H_pf.block(1,1,1,1)<< 1/(_c2_p_f_i[2]);
        H_pf.block(1,2,1,1)<< (-_c2_p_f_i[1])/(_c2_p_f_i[2]);


        Eigen::Matrix<double,3,30> H_wi;
        H_wi.setZero();
        H_wi.block(0,0,3,3) = -c_R_I*_1_R_2*skew_x(_1_p_f_i);
        H_wi.block(0,12,3,3) = c_R_I*N_R_2;
        H_wi.block(0,15,3,3)=  c_R_I*skew_x(_2_p_f_i);
        H_wi.block(0,27,3,3)= -c_R_I*N_R_2;

        Eigen::Matrix<double,1,30> H_i= HI*H_pf*H_wi;






        _jacobianOplusXi= H_i.block(0,0,1,15);
        _jacobianOplusXj= H_i.block(0,15,1,15);

        if (std::isnan(_jacobianOplusXi.norm()) || std::isnan(_jacobianOplusXj.norm())){
            cout << "Error in dense Jac" << endl;
            cout << "H_i" << endl << H_i << endl << endl << "HI" << endl << HI << endl << endl << "H_pf" << endl << H_pf << endl;
            cout << endl << "H_wi" << endl << H_wi << endl << endl << "_c2_p_f_i[2]" << endl << _c2_p_f_i[2] << endl;
            cout << "_c2_p_f_i_p" << endl << _c2_p_f_i_p << endl << endl << "_c1_p_f_i" << endl << _c1_p_f_i << endl;
            std::exit(1);}
    }



};


}

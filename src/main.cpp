
#include <iostream>
#include <core/Frame_Classes.h>
#include "core/Corresponder.h"
#include "core/ImageSim.h"


#include "core/Frame_Classes.h"
#include "core/SFMgraph.h"
#include <Eigen/Dense>

using namespace MAST;
using namespace std;
#include "core/Initilizer.h"


int main() {

    Initilizer* Init= new Initilizer();


    /*Initilizer test = Initilizer();
    test.testing();*/

    Eigen::Matrix<double,2,2> ImageInfo= Eigen::MatrixXd::Identity(2,2);

    Eigen::Matrix<double,2,2> SonarInfo= Eigen::MatrixXd::Zero(2,2);

    SonarInfo(0,0) = 400;

    SonarInfo(0,0) = 1;

    Eigen::Matrix<double,7,1> cam_check;






    SFMgraph* Graph= new SFMgraph();

     vector< pair< Eigen::Matrix<double,3,3> , pair<int,int> > > measurements;

        Eigen::Matrix<double,3,3> K;
        K.setZero();

        K(0,0) = 466.6599239525347;

        K(0,2) = 408.15836612911477;

        K(1,1) = 465.8386070513852;
        K(1,2) = 243.27181754452832;

        K(2,2) = 1.0;

    cout << "main 41" << endl;

        vector<Camera*> cam_vec=  CamAsRandomizedPolygonSim(20, 8*sqrt(2)*M_PI/20, -4, -4 );


    cout << cam_vec.size() << endl;

    cout << "main 45" << endl;





    cout << "main 49" << endl;

    Eigen::Matrix<double, 3, 3> R_C_to_S;

    Eigen::Matrix<double,3,3> Ryneg90;

    Ryneg90 <<      0, 0, -1,
                    0, 1,  0,
                    1, 0,  0;

    Eigen::Matrix<double,3,3> Rxneg90;

    Rxneg90<<       1, 0,0,
                    0, 0, 1,
                    0, -1, 0;

    Eigen::Matrix<double, 3, 1> p_c_in_s;


    Eigen::Matrix<double,3,3> Reps;

    Reps<<   cos(.1), -sin(.1),0,
            sin(.1), cos(.1), 0,
            0, 0, 1;



    p_c_in_s << 0, 0, 0;


    Eigen::Matrix<double, 3, 1> p_c_in_s_err;

    p_c_in_s_err << .02, -.04, .03;


        for (int i=0; i < cam_vec.size(); i++){
            Camera* cam_i= cam_vec[i];
            cam_i->initialized=false;
            cam_i->K= K;
            cam_i->width= 640;
            cam_i->height= 480;
            cam_i->r_max=10;
            cam_i->th_max= M_PI_2/2;
            cam_i->psi_max=M_PI_2/2;
            cam_i->R_C_to_S= (Ryneg90*Rxneg90).transpose();
            cam_i->p_c_in_s= p_c_in_s;

        }

    cout << "main 65" << endl;


    JPL7* Calib= new JPL7();

    Calib->setId(0);

    //Calib->setFixed(true);

    int edgeID=0;

    Eigen::Matrix<double,7,1> calib_meas;

    calib_meas.block(0,0,4,1)= rot_2_quat((Ryneg90*Rxneg90).transpose()*Reps);

    calib_meas.block(4,0,3,1)= p_c_in_s_err;

    Calib->setEstimate(calib_meas);

    bool good= Graph->graph.addVertex(Calib);

    PriorEdge* CalibPrior = new PriorEdge();

    CalibPrior->setId(edgeID);

    edgeID++;

    CalibPrior->resize(1);

    CalibPrior->prior_meas= calib_meas;

    Eigen::Matrix<double,6,6> PriInf= Eigen::MatrixXd::Zero(6,6);

    PriInf.block(0,0,3,3)= 100*Eigen::MatrixXd::Identity(3,3);

    PriInf.block(3,3,3,3)= 400*Eigen::MatrixXd::Identity(3,3);

    CalibPrior->setInformation(Eigen::MatrixXd::Identity(6,6));

    assert(Calib != nullptr);

    cout << Calib->graph() << endl;

    cout << good << endl;

    good=true;

    if (!Calib->graph()){
        cout << "Calib isnt in graph!" << endl;
        std::exit(1);

    }

    CalibPrior->setVertex(0, Calib);



    good= Graph->graph.addEdge(CalibPrior);


    if (!good){
        cout << "Err 152" << endl;
        std::exit(1);

    }






    //Ok here we are gonna Generate the features
    //Simulation code
    //



    ImageSim(cam_vec, 20);

    //std::exit(1);

    cout << "main 74" << endl;


       Eigen::Matrix<double,3,3> Kinv= K.inverse();


    cout << "main 80" << endl;


    int id_count=1;

    int added_vert_count=0;


        for (int t=0; t< cam_vec.size(); t++){





            cout << "main 145" << endl;

            Camera* cam_t= cam_vec[t];

            cout << "main 146" << endl;

            vector<Feature_Class*> feats_in_cam= cam_t->feature_list;

            cout << feats_in_cam.size() << endl;

            cout << "main 150" << endl;

            cout << "cam- " << t << " has this many features- " << feats_in_cam.size() << endl;

            vector<Feature_Class*> image_only_features;

            vector<Feature_Class*> sonar_only_features;

            vector<Feature_Class*> image_and_sonar_features;







            for (int f=0; f < feats_in_cam.size(); f++){
                Feature_Class* feat= feats_in_cam[f];
                if (feat->uv_locations[t].first != -1){
                    if (feat->r_theta_values[t].first != -1){
                        image_and_sonar_features.push_back(feat);
                    }
                    else{
                        image_only_features.push_back(feat);
                    }
                }
                else{
                    sonar_only_features.push_back(feat);
                }
            }

            cout << "cam- " << t << " has this many image_only features- " << image_only_features.size() << endl;

            cout << "cam- " << t << " has this many sonar_only features- " << sonar_only_features.size() << endl;

            cout << "cam- " << t << " has this many image+sonar features- " << image_and_sonar_features.size() << endl;




            cam_t->camera_vertex->setId(id_count);
            id_count++;

            if (t==0){
                Eigen::Matrix<double,7,1> orig_state;
                orig_state << 0,0,0,1,0,0,0;

                cam_t->initialized=true;

                cam_t->camera_vertex->setEstimate(orig_state);

                good = Graph->graph.addVertex(cam_t->camera_vertex);

               /* cout << cam_t->camera_vertex->graph() << endl;

                cout << good << endl;*/


                cam_t->camera_vertex->setFixed(true);

                for (int i=0; i < image_and_sonar_features.size(); i++){
                    Feature_Class* feat_for_init = image_and_sonar_features[i];
                    Eigen::Matrix<double,3,1> uv;
                    pair<int,int> uvmeas= feat_for_init->uv_locations[0];
                    double r= feat_for_init->r_theta_values[0].first;
                    uv << uvmeas.first, uvmeas.second, 1.0;

                    feat_for_init->feature->setEstimate(Init->init_feat_cam2(
                            cv::Point(uvmeas.first, uvmeas.second), cv::Point(r, feat_for_init->r_theta_values[0].second),
                            Calib->estimate().block(0,0,4,1), Calib->estimate().block(4,0,3,1)));


                    cout << "Triangle Estimate" << endl << feat_for_init->feature->estimate() << endl << endl << "..........................." << endl << endl;


                    if (std::isnan(feat_for_init->feature->estimate().norm())){
                        cout << "BAD INITIAL" << endl;
                        std::exit(1);
                    }


                    feat_for_init->feature->setId(id_count);

                    id_count++;

                    Graph->graph.addVertex(feat_for_init->feature);


                    /*cout << Init->init_feat_cam2(
                            cv::Point(uvmeas.first, uvmeas.second), cv::Point(r, feat_for_init->r_theta_values[0].second),
                            Calib->estimate().block(0,0,4,1), Calib->estimate().block(4,0,3,1)) << endl << endl << ".............." << endl << endl;*/


                    feat_for_init->initialized=true;


                }


            }
            else {

                //First we must initialize camera
                int init_feats_with_rel_pose;
                vector<Eigen::Matrix<double, 3, 1>> ps_in_c;
                vector<Feature_Class *> feat_for_triang;
                vector<Feature *> feat_graph;
                for (int i = 0; i < image_and_sonar_features.size(); i++) {
                    Feature_Class *feat_for_init = image_and_sonar_features[i];

                    if (feat_for_init->initialized) {
                        Eigen::Matrix<double, 3, 1> uv;
                        pair<int, int> uvmeas = feat_for_init->uv_locations[t];
                        uv << uvmeas.first, uvmeas.second, 1.0;
                        uv = Kinv * uv;
                        Eigen::Matrix<double, 3, 1> p_f_in_c = Init->init_feat_cam2(
                                cv::Point(uvmeas.first, uvmeas.second), cv::Point(feat_for_init->r_theta_values[t].first, feat_for_init->r_theta_values[t].second),
                                Calib->estimate().block(0,0,4,1), Calib->estimate().block(4,0,3,1));




                            ps_in_c.push_back(p_f_in_c);
                            feat_for_triang.push_back(feat_for_init);
                            feat_graph.push_back(feat_for_init->feature);


                    }

                }
                if (ps_in_c.size() > 2) {


                    //cout << "cam FIND 20" << endl;
                    Graph->find_camera_from_features(cam_t->camera_vertex, ps_in_c, feat_for_triang);

                    good = Graph->graph.addVertex(cam_t->camera_vertex);

                    cam_t->initialized=true;



                }
            }

                    //cout << "main 323" << endl;

                    if (!cam_t->initialized){
                        cout << "DID NOT INITIALIZE" << endl;
                        std::exit(1);
                    }


                    Eigen::Matrix<double ,7,1> cam_est= cam_t->camera_vertex->estimate();


                    Eigen::Matrix<double,3,3> Rt= quat_2_Rot(cam_est.block(0,0,4,1));

                    Eigen::Matrix<double,3,1> pt= cam_est.block(4,0,3,1);


                    Eigen::Matrix<double,7,1> calib_est= Calib->estimate();

                    Eigen::Matrix<double,3,3> R_c_2_s= quat_2_Rot(calib_est.block(0,0,4,1));

                    Eigen::Matrix<double,3,1> p_c_in_s= calib_est.block(4,0,3,1);


                    cout << "main 346 " << endl;

                    cout << image_and_sonar_features.size() << endl;

                    for (int fo=0; fo< image_and_sonar_features.size();fo++){

                        //cout << "Here 352" << endl;

                        Feature_Class *feat_i = image_and_sonar_features[fo];

                        //cout << (feat_i->initialized) << endl;

                        double r_m= feat_i->r_theta_values[t].first;
                        double th_m= feat_i->r_theta_values[t].second;

                        Eigen::Matrix<double, 3, 1> uvi;
                        pair<int, int> uvimeas = feat_i->uv_locations[t];
                        uvi << uvimeas.first, uvimeas.second, 1.0;
                        uvi = Kinv * uvi;

                        if (!feat_i->initialized) {
                            //cout << "Begin initial" << endl;



                            Eigen::Matrix<double, 3, 1> p_f_in_ci = Init->init_feat_cam2(
                                    cv::Point(uvimeas.first, uvimeas.second), cv::Point(feat_i->r_theta_values[t].first, feat_i->r_theta_values[t].second),
                                    Calib->estimate().block(0,0,4,1), Calib->estimate().block(4,0,3,1));


                            //cout << "Triangle Estimate" << p_f_in_ci << endl << endl << "..........................." << endl << endl;


                            if (std::isnan(p_f_in_ci.norm())){
                                cout << "BAD INITIAL" << endl;
                                std::exit(1);
                            }

                            Eigen::Matrix<double, 3, 1> p_f_ing = Rt.transpose() * (p_f_in_ci - pt);

                            feat_i->initialized = true;

                            //cout << p_f_ing << endl << "//////////////////////////" << endl << endl;

                            feat_i->feature->setEstimate(p_f_ing);

                            feat_i->feature->setId(id_count);

                            id_count++;

                            Graph->graph.addVertex(feat_i->feature);

                            //cout << "added feature !???????????????????????????" << endl;
                        }

                        SonarEdge* s_edge= new SonarEdge();
                        s_edge->setId(edgeID);

                        edgeID++;
                        s_edge->resize(3);
                        s_edge->setInformation(SonarInfo);
                        assert(Calib != nullptr);
                        s_edge->setVertex(0, Calib);
                        assert(cam_t->camera_vertex != nullptr);
                        s_edge->setVertex(1, cam_t->camera_vertex);
                        assert(feat_i->feature != nullptr);
                        s_edge->setVertex(2, feat_i->feature);



                        Eigen::Matrix<double,2,1> r_th_m;
                        r_th_m << r_m, th_m;


                        //cout << "rth- " << endl << r_th_m << endl << endl << ".........." << endl;
                        s_edge->setMeasurement(r_th_m);
                        good = Graph->graph.addEdge(s_edge);

                        if ( !good){
                            cout << "ERrro 358" << endl;
                            std::exit(1);
                        }




                        ImageEdge* i_edge= new ImageEdge();
                        i_edge->setId(edgeID);

                        edgeID++;
                        i_edge->resize(2);
                        i_edge->setInformation(ImageInfo);
                        assert(cam_t->camera_vertex != nullptr);
                        i_edge->setVertex(0, cam_t->camera_vertex);
                        assert(feat_i->feature != nullptr);
                        i_edge->setVertex(1, feat_i->feature);

                        if ((i_edge->vertices().size() ==0) ||feat_i->initialized==false || cam_t->initialized==false || (!Calib) || (!cam_t->camera_vertex) ||(!feat_i->feature) ||(!i_edge->vertices()[0])){
                            cout << "ERrro 374" << endl;
                            std::exit(1);
                        }




                        Eigen::Matrix<double,2,1> uvm;

                        uvm << uvi(0,0), uvi(1,0);


                        //cout << "uvm- " << endl << uvm << endl << endl << ".........." << endl;

                        i_edge->setMeasurement(uvm);

                        good = Graph->graph.addEdge(i_edge);

                        if (!good){
                            cout << "Err 428" << endl;
                            cout << cam_t->camera_vertex->graph() << endl;

                            std::exit(1);

                        }



                        // Here we create new factors

                    }
                    for (int fy=0; fy< image_only_features.size();fy++) {
                        Feature_Class *feat_i = image_only_features[fy];
                        int other_cam = -2;
                        if (!feat_i->initialized) {
                            other_cam = -1;
                            for (int oc = 0; oc < t; oc++) {
                                if (feat_i->uv_locations[oc].first != -1) {
                                    other_cam = oc;
                                }
                            }
                            if (other_cam != -1) {
                                Camera *o_cam = cam_vec[other_cam];
                                Eigen::Matrix<double, 3, 1> uv;
                                uv << feat_i->uv_locations[other_cam].first, feat_i->uv_locations[other_cam].second, 1;
                                Eigen::Matrix<double, 3, 1> uvnorm = Kinv * uv;

                                Eigen::Matrix<double, 3, 1> uv2;
                                uv2 << feat_i->uv_locations[t].first, feat_i->uv_locations[t].second, 1;
                                Eigen::Matrix<double, 3, 1> uv2norm = Kinv * uv2;

                                Eigen::Matrix<double,3,1> p_f_in_c_o= Graph->triangulate_point(o_cam->camera_vertex, cam_t->camera_vertex,
                                                         cv::Point2d(uvnorm(0, 0), uvnorm(1, 0)),
                                                         cv::Point2d(uv2norm(0, 0), uv2norm(1, 0)));

                                feat_i->initialized = true;
                                feat_i->feature->setId(id_count);

                                Eigen::Matrix<double,3,3> R_g_2_o= quat_2_Rot(o_cam->camera_vertex->estimate().block(0,0,4,1));
                                Eigen::Matrix<double,3,1> p_o_in_g = o_cam->camera_vertex->estimate().block(4,0,3,1);


                                Eigen::Matrix<double,3,1> p_f_in_g= R_g_2_o*(p_f_in_c_o) + p_o_in_g;


                                //cout << "P FROM UV STUFF" << endl;
                                feat_i->feature->setEstimate(p_f_in_g);

                                //cout << p_f_in_g << endl;

                                //cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl << endl;

                                id_count++;
                                Graph->graph.addVertex(feat_i->feature);

                                //Add from old image

                                ImageEdge* i_edge1= new ImageEdge();
                                i_edge1->setId(edgeID);
                                edgeID++;
                                i_edge1->resize(2);
                                i_edge1->setInformation(ImageInfo);
                                assert(o_cam->camera_vertex != nullptr);
                                i_edge1->setVertex(0, o_cam->camera_vertex);
                                assert(feat_i->feature != nullptr);
                                i_edge1->setVertex(1, feat_i->feature);

                                if ((i_edge1->vertices().size() ==0) ||feat_i->initialized==false || o_cam->initialized==false || (!Calib) || (!cam_t->camera_vertex) ||(!feat_i->feature) || (!i_edge1->vertices()[0])){
                                    cout << "ERrro 439" << endl;
                                    std::exit(1);
                                }


                                Eigen::Matrix<double,2,1> uvm;

                                uvm << uvnorm(0,0), uvnorm(1,0);

                                i_edge1->setMeasurement(uvm);

                                good= Graph->graph.addEdge(i_edge1);

                                if (!good){
                                    cout << "Err 152" << endl;
                                    std::exit(1);

                                }


                            }
                            //Finish doing the triangulation
                        }
                        if (other_cam != -1) {



                            ImageEdge* i_edge2= new ImageEdge();
                            i_edge2->setId(edgeID);

                            edgeID++;
                            i_edge2->resize(2);
                            i_edge2->setInformation(ImageInfo);
                            assert(cam_t->camera_vertex != nullptr);
                            i_edge2->setVertex(0, cam_t->camera_vertex);
                            assert(feat_i->feature != nullptr);
                            i_edge2->setVertex(1, feat_i->feature);

                            if ((i_edge2->vertices().size() ==0) ||feat_i->initialized==false || cam_t->initialized==false || (!Calib) || (!cam_t->camera_vertex) ||(!feat_i->feature)  || (!i_edge2->vertices()[0])){
                                cout << other_cam << endl;
                                cout << "///////" << endl << (feat_i->initialized==false) << endl << (cam_t->initialized==false) << endl;
                                cout << "ERrro 457" << endl;
                                std::exit(1);
                            }


                            Eigen::Matrix<double,3,1> uvm;

                            uvm << feat_i->uv_locations[t].first, feat_i->uv_locations[t].second, 1;

                            uvm= Kinv*uvm;

                            i_edge2->setMeasurement(uvm.block(0,0,2,1));

                            good = Graph->graph.addEdge(i_edge2);

                            if (!good){
                                cout << "Err 543" << endl;
                                std::exit(1);

                            }



                        }
                    }


                        // Here we create new factors from both images and sonar

            Graph->graph.setVerbose(true); // printOptimizationInfo








            Graph->graph.initializeOptimization();


                    cout << "Number of edges- " <<  Graph->graph.edges().size() << endl;
                    cout << "Number of verts- " << Graph->graph.vertices().size() << endl;
                    cout << "Iteration- " << t << endl;

            cout << "Here 593" << endl;


                    cout << "Hello 618" << endl;
                    Graph->graph.optimize(10);

            Eigen::Matrix<double,3,3> R_g_2_0= cam_vec[0]->R_G_to_C;

            Eigen::Matrix<double,3,1> p_0_in_g= cam_vec[0]->p_G_to_C;

            for (int c=0; c< t; c++){
                Camera* cam_c= cam_vec[c];
                Eigen::Matrix<double,3,3> R_0_2_c= quat_2_Rot(cam_vec[c]->camera_vertex->estimate().block(0,0,4,1));
                Eigen::Matrix<double,3,1> p_c_in_0  =cam_vec[c]->camera_vertex->estimate().block(4,0,3,1);
                cout << "Camera " << c << " true values- " << endl << "R_true- " << endl << cam_vec[c]->R_G_to_C << endl << "Ptrue" << endl << cam_vec[c]->p_G_to_C << endl << endl;
                cout << "Camera " << c << " est values- " << endl << "R_est- " << endl << R_0_2_c*R_g_2_0 << endl << "Ptrue" << endl << (R_g_2_0.transpose()*(p_c_in_0)+p_0_in_g) << endl << endl << "////////////////////////" << endl;
            }



            cout << "Hello 620" << endl;

            bool bad=false;
            /*for (int f=0; f< cam_t->feature_list.size(); f++){
                if (cam_t->feature_list[f]->feature->estimate().norm() > 1000){
                    bad= true;
                }
                cout << "feature " << f << endl << cam_t->feature_list[f]->feature->estimate() << endl << endl << ".............." << endl << endl;
            }*/

            if (cam_t->camera_vertex->estimate().norm() >1000){
                bad =true;
            }



            if (bad == true){
                cout << cam_t->camera_vertex->estimate() << endl;
                cout << t << endl;
            std::exit(1);}


                    cout << cam_t->camera_vertex->estimate() << endl << "////////////////////////////////////////////////" << endl << endl;





            }

    Eigen::Matrix<double,3,3> R_g_2_0= cam_vec[0]->R_G_to_C;

    Eigen::Matrix<double,3,1> p_0_in_g= cam_vec[0]->p_G_to_C;

    for (int c=0; c< cam_vec.size(); c++){
        Camera* cam_c= cam_vec[c];
        Eigen::Matrix<double,3,3> R_0_2_c= quat_2_Rot(cam_vec[c]->camera_vertex->estimate().block(0,0,4,1));
        Eigen::Matrix<double,3,1> p_c_in_0  =cam_vec[c]->camera_vertex->estimate().block(4,0,3,1);
        cout << "Camera " << c << " true values- " << endl << "R_true- " << endl << cam_vec[c]->R_G_to_C << endl << "Ptrue" << endl << cam_vec[c]->p_G_to_C << endl << endl;
        cout << "Camera " << c << " est values- " << endl << "R_est- " << endl << R_0_2_c*R_g_2_0 << endl << "Ptrue" << endl << (R_g_2_0.transpose()*(p_c_in_0)+p_0_in_g) << endl << endl << "////////////////////////" << endl;
    }

    cout << "Calib" << endl << Calib->estimate() << endl;



    cout << "//////////////////////////////" << endl << endl;
    //cout << R_C_to_S << endl;
    cout << "Calibtrue" << endl << rot_2_quat((Ryneg90*Rxneg90).transpose()) << endl <<0 << endl <<0 << endl << 0 << endl;



    cout << "Calibbegin" << endl << calib_meas << endl << endl;







}

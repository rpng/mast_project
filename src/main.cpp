
#include <iostream>
#include <core/Frame_Classes.h>
#include "core/Corresponder.h"
#include "core/ImageSim.h"

#include "core/Frame_Classes.h
#include "core/SFMgraph.h"
#include <Eigen/Dense>

using namespace MAST;
using namespace std;
=======
#include "core/Initilizer.h"

using namespace MAST;

int main() {


    Initilizer test = Initilizer();
    test.testing();


        Eigen::Matrix<double,3,3> K;

        Eigen::Matrix<double,3,3> Kinv= K.inverse();


        vector<Camera*> cam_vec;

        SFMgraph* Graph= new SFMgraph();

        //Ok here we are gonna Generate the features
        //Simulation code
        //

        for (int t=0; t< cam_vec.size(); t++){
            Camera* cam_t= cam_vec[t];
            vector<Feature_Class*> feats_in_cam= cam_t->feature_list;

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

            if (t==0){
                Eigen::Matrix<double,7,1> orig_state;
                orig_state << 0,0,0,1,0,0,0;
                cam_t->camera_vertex->setEstimate(orig_state);
                for (int i=0; i < image_and_sonar_features.size(); i++){
                    Feature_Class* feat_for_init = image_and_sonar_features[i];
                    Eigen::Matrix<double,3,1> uv;
                    pair<int,int> uvmeas= feat_for_init->uv_locations[0];
                    uv << uvmeas.first, uvmeas.second, 1.0;
                    uv= Kinv*uv;
                    feat_for_init->feature->setEstimate(Graph->find_feature_from_image_and_sonar (cam_t->camera_vertex,  uv, feat_for_init->r_theta_values[0].first));


                }


            }
            else{

                //First we must initialize camera
                int init_feats_with_rel_pose;
                vector <Eigen::Matrix<double,3,1>> ps_in_c;
                vector<Feature_Class*> feat_for_triang;
                vector<Feature*> feat_graph;
                for (int i=0; i < image_and_sonar_features.size(); i++) {
                    Feature_Class *feat_for_init = image_and_sonar_features[i];

                    if (feat_for_init->initialized) {
                        Eigen::Matrix<double, 3, 1> uv;
                        pair<int, int> uvmeas = feat_for_init->uv_locations[0];
                        uv << uvmeas.first, uvmeas.second, 1.0;
                        uv = Kinv * uv;
                        Eigen::Matrix<double, 3, 1> p_f_in_c = Graph->find_feature_from_image_and_sonar(
                                cam_t->camera_vertex, uv, feat_for_init->r_theta_values[0].first);
                        if (ps_in_c.size() <3) {
                            ps_in_c.push_back(p_f_in_c);
                            feat_for_triang.push_back(feat_for_init);
                            feat_graph.push_back(feat_for_init->feature);
                        }

                    }

                }
                if (ps_in_c.size() >3){


                    for (int fo=0; fo< image_and_sonar_features.size();fo++){
                        Feature_Class *feat_i = image_and_sonar_features[fo];
                        Eigen::Matrix<double, 3, 1> uvi;
                        pair<int, int> uvimeas = feat_i->uv_locations[0];
                        uvi << uvimeas.first, uvimeas.second, 1.0;
                        uvi = Kinv * uvi;
                        Eigen::Matrix<double, 3, 1> p_f_in_ci = Graph->find_feature_from_image_and_sonar(
                                cam_t->camera_vertex, uvi, feat_i->r_theta_values[0].first);


                    }
                    for (int fy=0; fy< image_only_features.size();fy++) {
                        Feature_Class *feat_i = image_only_features[fy];
                        //Finish doing the triangulation
                    }
                }


            }


        }


}


#include <iostream>
#include <core/Frame_Classes.h>
#include "core/Corresponder.h"
#include "core/ImageSim.h"
#include "core/Frame_Classes.h
#include "core/SFMgraph.h"
#include <Eigen/Dense>

using namespace MAST;
using namespace std;

    int main() {

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
                    Graph->find_feature_from_image_and_sonar (cam_t->camera_vertex,  uv, feat_for_init->r_theta_values[0].first);

                }


            }


        }


    }

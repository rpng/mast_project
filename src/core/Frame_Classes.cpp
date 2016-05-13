//
// Created by keck on 5/4/16.
//

//
// Created by keck on 5/4/16.
//


#include <iostream>
#include <ctime>
#include <cstdlib>
#include "core/Frame_Classes.h"

using namespace std;

namespace MAST {

    Feature_Class::Feature_Class() {
        this->feature= new Feature();

    }

    Camera::Camera() {
        this->camera_vertex= new JPL7();
    }

}
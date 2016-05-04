cmake_minimum_required(VERSION 2.8 FATAL_ERROR)



project(mast_project)


SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)


find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
		include
		${EIGEN3_INCLUDE_DIR}
		${OpenCV_INCLUDE_DIRS}
)

#add_executable(ImageSim ImageSim.cpp)
add_executable(mast_project
		src/main.cpp
		src/core/SFMgraph.cpp
		src/types/JPL7.cpp
)

target_link_libraries (mast_project ${OpenCV_LIBS})





cmake_minimum_required(VERSION 2.8 FATAL_ERROR)



project(mast_project)


SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)


find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 REQUIRED)
add_executable (ImageSim ImageSim.cpp) 

include_directories(
	include
	${EIGEN3_INCLUDE_DIR}
	${OpenCV_INCLUDE_DIRS}
)


target_link_libraries (mast ${OpenCV_LIBS} )





cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(mast_project)


# Try to compile with c++11
# http://stackoverflow.com/a/25836953
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
	message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(G2O REQUIRED)
find_package(SuiteParse REQUIRED)
find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
		include
		${CSPARSE_INCLUDE_DIR} #Has been set by SuiteParse
		${CHOLMOD_INCLUDE_DIR} #Has been set by SuiteParse
		${G2O_INCLUDE_DIRS}
		${EIGEN3_INCLUDE_DIR}
		${OpenCV_INCLUDE_DIRS}
)

# Create the binary file
add_executable(mast_project
		src/main.cpp
		src/core/SFMgraph.cpp
		src/types/JPL7.cpp
		src/core/Corresponder.cpp
		src/core/ImageSim.cpp
		src/core/Frame_Classes.cpp
		src/core/Initilizer.cpp
)
target_link_libraries (mast_project ${OpenCV_LIBS} ${G2O_LIBRARIES} csparse cxsparse)





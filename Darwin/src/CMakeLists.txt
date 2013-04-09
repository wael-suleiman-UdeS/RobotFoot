# APP GEI 744, Copyright 2013, Wael Suleiman
 
cmake_minimum_required (VERSION 2.6) 

project (GEI744_labo3) 

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(Eigen3 REQUIRED)

#SET( EIGEN3_INCLUDE_DIR "/opt/local/include/eigen3/" )
#SET( DARWIN_INCLUDE_DIR "" )

include_directories(${EIGEN3_INCLUDE_DIR})

set(SRCS
 main.cpp
matrixpinv.cpp
WalkStatus.cpp
)
add_executable(App2 ${SRCS})

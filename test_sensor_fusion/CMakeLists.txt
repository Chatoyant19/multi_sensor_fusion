cmake_minimum_required( VERSION 2.8 )
project( test_sensor_fusion )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall" )

include_directories( "/usr/include/eigen3" )

add_executable( test_sensor_fusion 
  main.cpp 
  kalman.cpp
  sensor_fusion.cpp)
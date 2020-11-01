cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(icp_test)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
${PCL_INCLUDE_DIRS} 
${EIGEN3_INCLUDE_DIR}
)

add_executable(icp_test icp.cpp)
target_link_libraries (icp_test 
${PCL_LIBRARIES}
)


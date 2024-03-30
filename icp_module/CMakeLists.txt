cmake_minimum_required(VERSION 3.5)
project(icp_p2p)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DELPP_NO_DEFAULT_LOG_FILE")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  tf
  tf2 
  tf2_ros 
  message_generation
)

include_directories( ${catkin_INCLUDE_DIRS})


#找到tbb, glog
find_package(TBB REQUIRED)
find_package(Glog REQUIRED)
include_directories(${Glog_INCLUDE_DIRS})

# 查找YAML-CPP库
find_package(yaml-cpp REQUIRED)
find_package(Eigen3)
find_package(Sophus)
find_package( PCL 1.8 REQUIRED)#解决版本问题
find_package(Boost REQUIRED COMPONENTS system thread)#for boost
 
include_directories( ${PCL_INCLUDE_DIRS})
include_directories(${Sophus_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

add_executable(icp_pp src/icp_pp.cpp)
target_link_libraries(icp_pp  
                ${PCL_LIBRARIES}
                glog 
                fmt 
                ${Sophus_LIBRARIES}
                ${YAML_CPP_LIBRARIES}
                ${Boost_LIBRARIES}
                )
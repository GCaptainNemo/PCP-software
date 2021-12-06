cmake_minimum_required(VERSION 3.0.2)
project(calib_data)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  image_transport
  sensor_msgs
  livox_ros_driver
)
catkin_package(

)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io visualization )
set(EIGEN3_INCLUDE_DIR /usr/local/include/include/eigen3)
include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ./include/ ../common_utils/)

add_executable(calib_node src/calib.cpp ../common_utils/io_utils.cpp)
target_link_libraries(calib_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIB_DIR} ${PCL_LIBRARIES})

# add_executable(rgb_snapshot src/rgb_snapshot.cpp)
# target_link_libraries(rgb_snapshot ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


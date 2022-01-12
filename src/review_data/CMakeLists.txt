cmake_minimum_required(VERSION 3.0.2)
project(review_data)
add_compile_options(-std=c++14)
#set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS "--std=c++14")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
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

add_executable(review_color_lidar_node src/review_color_lidar_node.cpp src/utils.cpp ../common_utils/io_utils.cpp)
target_link_libraries(review_color_lidar_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIB_DIR} ${PCL_LIBRARIES})

# mono calib(circle grid)
add_executable(mono_calib src/calib_src/src/circle_board.cpp src/calib_src/src/blob_detector.cpp)
target_link_libraries(mono_calib ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )
target_include_directories(mono_calib PRIVATE src/calib_src/include)

# stero calib
add_executable(stereo_calib src/calib_src/src/stereo_calib.cpp 
			    src/calib_src/src/blob_detector.cpp)
target_link_libraries(stereo_calib ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )
target_include_directories(stereo_calib PRIVATE src/calib_src/include)



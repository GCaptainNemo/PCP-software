cmake_minimum_required(VERSION 3.0.2)
project(collect_data)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  image_transport
)
catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)
find_package(OpenCV)

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(ir_snapshot src/ir_snapshot.cpp)
target_link_libraries(ir_snapshot ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(rgb_snapshot src/rgb_snapshot.cpp)
target_link_libraries(rgb_snapshot ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


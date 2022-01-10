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
  message_generation
)

find_package(OpenCV)


add_message_files(FILES self_image.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)


include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(ir_rgb_snapshot src/ir_rgb_snapshot.cpp)
target_link_libraries(ir_rgb_snapshot ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(ir_rgb_remap src/ir_rgb_remap.cpp)
target_link_libraries(ir_rgb_remap ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(remap_test src/remap_test.cpp)
target_link_libraries(remap_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <opencv/highgui.h>
#include <unistd.h>
#include <stdio.h>
#include<sys/types.h>
#include<dirent.h>

#define ROS_MIN_MAJOR 1
#define ROS_MIN_MINOR 8
#define ROS_MIN_PATCH 16

#if ROS_VERSION_MINIMUM(ROS_MIN_MAJOR, ROS_MIN_MINOR, ROS_MIN_PATCH)
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/CvBridge.h>
#endif
#include <collect_data/self_image.h>


cv_bridge::CvImagePtr ir_cv_ptr, rgb_cv_ptr;

ros::Publisher ir_image_pub, rgb_image_pub;

void ir_imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  try
  {
    ir_cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat ir_out_img = ir_cv_ptr->image;
    cv::Size img_size = ir_out_img.size();
    collect_data::self_image self_ir_msg;
    self_ir_msg.height = img_size.height;
    self_ir_msg.width = img_size.width;
    self_ir_msg.channels = ir_out_img.channels();
    int size = self_ir_msg.height * self_ir_msg.width * self_ir_msg.channels;
    // self_ir_msg.data.resize(size);
    // for (int i = 0; i < size; ++i){
    //   self_ir_msg.data[i] = ir_out_img.data[i];
    // }
    self_ir_msg.data = std::vector<uint8_t>(ir_out_img.data, ir_out_img.data + size);
    ir_image_pub.publish(self_ir_msg);
    printf("width, height, channels = (%d, %d, %d)\n", img_size.width, img_size.height, self_ir_msg.channels);
    return;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    exit(0);
  }
}


void rgb_imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  try
  {
    rgb_cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat rgb_out_img = rgb_cv_ptr->image;
    cv::Size img_size = rgb_out_img.size();
    collect_data::self_image self_rgb_msg;
    self_rgb_msg.height = img_size.height;
    self_rgb_msg.width = img_size.width;
    self_rgb_msg.channels = rgb_out_img.channels();
    int size = self_rgb_msg.height * self_rgb_msg.width * self_rgb_msg.channels;
    // self_rgb_msg.data.resize(size);
    // for (int i = 0; i < size; ++i){
    //   self_rgb_msg.data[i] = rgb_out_img.data[i];
    // }
    
    self_rgb_msg.data = std::vector<uint8_t>(rgb_out_img.data, rgb_out_img.data + size);
    rgb_image_pub.publish(self_rgb_msg);
    printf("width, height = (%d, %d)\n", img_size.width, img_size.height);

    return;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    exit(0);
  }
}



int main(int argc, char **argv) {
  ros::init(argc, argv, "ir_rgb_remap");
  ros::NodeHandle nh;
 
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber ir_sub = it.subscribe("/usb_cam/image_raw", 10, ir_imageCallback);
  image_transport::Subscriber rgb_sub = it.subscribe("/hik_cam_node/hik_camera", 10, rgb_imageCallback);

  ir_image_pub = nh.advertise<collect_data::self_image>("/ir_remap", 10);    
  rgb_image_pub = nh.advertise<collect_data::self_image>("/rgb_remap", 10);
  ros::spin();
  return 0;
}



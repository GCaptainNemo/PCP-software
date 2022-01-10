#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <opencv/highgui.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/types.h>
#include <dirent.h>
#include <algorithm>

#define ROS_MIN_MAJOR 1
#define ROS_MIN_MINOR 8
#define ROS_MIN_PATCH 16

#if ROS_VERSION_MINIMUM(ROS_MIN_MAJOR, ROS_MIN_MINOR, ROS_MIN_PATCH)
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/CvBridge.h>
#endif
#include <collect_data/self_image.h>


 
void ir_imageCallback(const collect_data::self_image::ConstPtr & msg) {
  try
  {  
    // uchar a[msg.height * msg.width];
    // std::copy(msg.data.begin(), msg.data.end(), a);  
    cv::Mat ir_img(msg->height, msg->width, CV_8UC1, (void *)&msg->data[0]);
    // for(int i = 0; i < msg.height; ++i)
    // {
    //   for(int j = 0; j < msg.width; ++j)
    //   {
    //     ir_img.at<uchar>(i, j) = msg.data[i * msg.width + j];
    //   }
    // }
    cv::imshow("ir", ir_img);
    cv::waitKey(0);
    
    return;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    exit(0);
  }
}


void rgb_imageCallback(const collect_data::self_image::ConstPtr& msg) {
  try
  {
    // uchar a[msg.height * msg.width * msg.channels];
    // std::copy(msg.data.begin(), msg.data.end(), a) ;
    cv::Mat rgb_img(msg->height, msg->width, CV_8UC3, (void *)&msg->data[0]);
    // for(int i = 0; i < msg.height; ++i)
    // {
    //   for(int j = 0; j < msg.width; ++j)
    //   {
    //     rgb_img.at<cv::Vec3b>(i, j)[0] = msg.data[(i * msg.width + j) * 3];
    //     rgb_img.at<cv::Vec3b>(i, j)[1] = msg.data[(i * msg.width + j) * 3 + 1];
    //     rgb_img.at<cv::Vec3b>(i, j)[2] = msg.data[(i * msg.width + j) * 3 + 2];
      
    //   }
    // }
    cv::imshow("rgb", rgb_img);
    cv::waitKey(0);
    return;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    exit(0);
  }
}



int main(int argc, char **argv) {
  ros::init(argc, argv, "remap_test");
  ros::NodeHandle nh;
  ros::Subscriber ir_sub_, rgb_sub_;
  ir_sub_ = nh.subscribe<collect_data::self_image>("/ir_remap", 1, ir_imageCallback);
  rgb_sub_ = nh.subscribe<collect_data::self_image>("/rgb_remap", 1, rgb_imageCallback);

  // image_transport::ImageTransport it(nh);
  // image_transport::Subscriber ir_sub = it.subscribe("/usb_cam/image_raw", 10, ir_imageCallback);
  // image_transport::Subscriber rgb_sub = it.subscribe("/hik_cam_node/hik_camera", 10, rgb_imageCallback);

  // ir_image_pub = nh.advertise<collect_data::self_image>("/ir_remap", 10);    
  // rgb_image_pub = nh.advertise<collect_data::self_image>("/rgb_remap", 10);
  ros::spin();
  return 0;
}




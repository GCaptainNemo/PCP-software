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
#include <thread>

#define ROS_MIN_MAJOR 1
#define ROS_MIN_MINOR 8
#define ROS_MIN_PATCH 16

#if ROS_VERSION_MINIMUM(ROS_MIN_MAJOR, ROS_MIN_MINOR, ROS_MIN_PATCH)
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/CvBridge.h>
#endif



std::string ir_file_name, rgb_file_name;
bool save_ir_flag = true;
bool save_rgb_flag = true;

void ir_callback(const sensor_msgs::ImageConstPtr& msg) {
  try
  {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    if (save_ir_flag)
    {
      cv::imwrite(ir_file_name, cv_ptr->image);
      save_ir_flag = false;
    }
    else if(!save_rgb_flag){
      exit(0);
    }
    return;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    exit(0);
  }
}


void rgb_callback(const sensor_msgs::ImageConstPtr& msg) {
  try
  {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    if (save_rgb_flag)
    {
      cv::imwrite(rgb_file_name, cv_ptr->image);
      save_rgb_flag = false;
    }
    else if(!save_ir_flag){
      exit(0);
    }
    return;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    exit(0);
  }
}

void rgb_snapshot(){


}

int IsFolderExist(const char* path)
{
    DIR *dp;
    if ((dp = opendir(path)) == NULL)
    {
        return 0;
    }
 
    closedir(dp);
    return -1;
}

int IsFileExist(const char* path)
{
    return !access(path, F_OK);
}

void getFileName(const std::string & save_file_dir, const std::string & prefix)
{
      
  int index = 100;
  
  
  while (true)
  {
      ir_file_name = save_file_dir + prefix + std::string("_") + std::to_string(index) + std::string("__ir.jpg");
      rgb_file_name = save_file_dir + prefix + std::string("_") + std::to_string(index) + std::string("__rgb.jpg");

      ROS_INFO("file_name = %s", ir_file_name.c_str());
      if (IsFileExist(ir_file_name.c_str()) || IsFileExist(rgb_file_name.c_str()))
      {
        index += 1; 
        ir_file_name = save_file_dir + prefix + std::string("_") + std::to_string(index) + std::string("__ir.jpg");
        rgb_file_name = save_file_dir + prefix + std::string("_") + std::to_string(index) + std::string("__rgb.jpg");
        break;
      }
      else{
        index -= 1; 
      }
  }
  ROS_INFO("name = %s", ir_file_name.c_str());
}



int main(int argc, char **argv) {
  if (argc != 2){ROS_INFO("please enter output option (calib or data)!!"); return -1;}
  const char * output_option = argv[1];
  
  ros::init(argc, argv, "ir_snapshot");
  ros::NodeHandle nh;
  std::string save_dir;
  std::string prefix;
  nh.getParam("save_dir",save_dir);
  nh.getParam("prefix", prefix);
  ROS_INFO("IR snapshot node save_dir = %s", save_dir.c_str());
  if(!IsFolderExist(save_dir.c_str())){ROS_ERROR("save dir doesn't exist"); return -1;}
  ROS_INFO("save dir exists!!");
  
  std::string file_dir;
  if(strcmp(output_option, "calib") == 0){
     // calib 模式下，子前缀会动态更新
     getFileName(save_dir, prefix);
  }
  else if(strcmp(output_option, "data") == 0){
      ir_file_name = save_dir + prefix + "_ir.jpg";
      rgb_file_name = save_dir + prefix + "_rgb.jpg";
  }
  ROS_INFO("IR snapshot node file_dir = %s", file_dir.c_str());

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber ir_sub = it.subscribe("/usb_cam/image_raw", 10, ir_callback);
  image_transport::Subscriber rgb_sub = it.subscribe("/hik_cam_node/hik_camera", 10, rgb_callback);
  ros::spin();
  // while (true)
  // {
  //   cv::Size img_size = out_img.size();
  //   if (img_size.width != 0){break;}
  //   ros::spinOnce();
  // }
  // cv::imwrite(file_dir, out_img);
  return 0;
}


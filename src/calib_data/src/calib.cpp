#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


#include <sensor_msgs/PointCloud2.h>
#include "livox_ros_driver/CustomMsg.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h> 
#include <pcl/visualization/cloud_viewer.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <opencv/highgui.h>
#include <stdio.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>


#include <Eigen/Dense>
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>

#include "io_utils.h"

#define ROS_MIN_MAJOR 1
#define ROS_MIN_MINOR 8
#define ROS_MIN_PATCH 16

#if ROS_VERSION_MINIMUM(ROS_MIN_MAJOR, ROS_MIN_MINOR, ROS_MIN_PATCH)
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/CvBridge.h>
#endif

#define foreach BOOST_FOREACH
typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;


const bool IS_FILTER = true;
const bool IS_IMAGE_CORRECTION = true;



class ImageLidarCalib
{
public:
    cv::Mat transform_matrix;    /**< from globol to image coordinate */
    cv::Mat intrinsic_matrix;    /**< from local to image coordinate  */
	  cv::Mat extrinsic_matrix;    /**< from global to local coordinate */
	  cv::Mat dist_matrix;         /**< dist parameters  */
    int img_height;
    int img_width;
    
    
    cv::Mat img;
    cv::Mat range_img;
    cv::Mat mask_img;       /**count reprojection times*/

public:
    PointCloudXYZI::Ptr pc_ptr_xyzi; 
    PointCloudXYZRGB::Ptr pc_ptr_xyzrgb;

public:
    ros::NodeHandle node_handle;
    std::string lidar_dir;
    std::string img_dir;
    
public:
  ImageLidarCalib()
  {
    ROS_INFO("------------ intialize ----------\n");
    pc_ptr_xyzi.reset(new PointCloudXYZI());
    pc_ptr_xyzrgb.reset(new PointCloudXYZRGB());    
    this->set_param();
  };

   ~ImageLidarCalib()
  {
  };
  
  // get camera parameters
  void set_param();
  
  // 
  int get_data();

  // read visual image and infrared image 
  void get_img();

  // get lidar 
  void get_lidar();  

  // get lidar, infrared and visual image and fusion(get colored lidar)
  int calib();

};


void ImageLidarCalib::set_param() 
{
  // extrinsic matrix parameters
  XmlRpc::XmlRpcValue param_list;
  std::vector<double> Extrin_matrix;
  if(!node_handle.getParam("/calib_node/CameraExtrinsicMat/data", param_list))
      ROS_ERROR("Failed to get extrinsic parameter.");
  ROS_INFO("\n get extrinsic parameter:");
  for (size_t i = 0; i < param_list.size(); ++i) 
  {
      XmlRpc::XmlRpcValue tmp_value = param_list[i];
      if(tmp_value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
      {    
        Extrin_matrix.push_back(double(tmp_value));
        ROS_INFO("PARAME SIZE = %f", double(tmp_value));
      }
  }
  // Intrinsic matrix parameters
  std::vector<double> Intrinsic;
  if(!node_handle.getParam("/calib_node/CameraMat/data", param_list))
      ROS_ERROR("Failed to get extrinsic parameter.");
  ROS_INFO("\n get intrinsic parameter:");
  for (size_t i = 0; i < param_list.size(); ++i) 
  {
      XmlRpc::XmlRpcValue tmp_value = param_list[i];
      if(tmp_value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
      {    
        Intrinsic.push_back(double(tmp_value));
        ROS_INFO("PARAME SIZE = %f", double(tmp_value));
      }
  }


  // 5 distortion parameters
  std::vector<double> dist;
  if(!node_handle.getParam("/calib_node/DistCoeff/data", param_list))
      ROS_ERROR("Failed to get extrinsic parameter.");
  ROS_INFO("\n get distortion parameter:");
  for (size_t i = 0; i < param_list.size(); ++i) 
  {
      XmlRpc::XmlRpcValue tmp_value = param_list[i];
      if(tmp_value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
      {    
        dist.push_back(double(tmp_value));
        ROS_INFO("PARAME SIZE = %f", double(tmp_value));
      }
  }
  
  // img size
  std::vector<int> img_size;
  if(!node_handle.getParam("/calib_node/ImageSize", param_list))
      ROS_ERROR("Failed to get extrinsic parameter.");
  ROS_INFO("\n get image size:");
  for (size_t i = 0; i < param_list.size(); ++i) 
  {
      XmlRpc::XmlRpcValue tmp_value = param_list[i];
      if(tmp_value.getType() == XmlRpc::XmlRpcValue::TypeInt)
      {    
        img_size.push_back(int(tmp_value));
        ROS_INFO("PARAME SIZE = %d", int(tmp_value));
      }
  }
  img_width = img_size[0];
  img_height = img_size[1];
  range_img = cv::Mat::zeros(img_height, img_width, CV_32FC1);
  mask_img = cv::Mat::zeros(img_height, img_width, CV_8UC1);
  // convert cv::Mat
  cv::Mat dist_array(5, 1, CV_64F, &dist[0]);
  this->dist_matrix = dist_array.clone();
  
  cv::Mat Int(3, 3, CV_64F, &Intrinsic[0]);
  this->intrinsic_matrix = Int.clone();

  
  cv::Mat ext_(4, 4, CV_64F, &Extrin_matrix[0]);
  cv::Mat invRt = ext_(cv::Rect(0, 0, 3, 3));
  cv::Mat R = invRt.t();
  cv::Mat invT = -R * ext_(cv::Rect(3, 0, 1, 3));
  cv::hconcat(R, invT, this->extrinsic_matrix);
  // transform matrix: from global coordinate to image coordinate
  this->transform_matrix = Int * this->extrinsic_matrix;
}


void ImageLidarCalib::get_img()
{
  this->img = cv::imread(this->img_dir, -1);
  if (IS_IMAGE_CORRECTION){
    cv::Mat undist_img;
    cv::undistort(img, undist_img, this->intrinsic_matrix, this->dist_matrix);
    img = undist_img.clone();
  }
  
};


void ImageLidarCalib::get_lidar()
{
    // cannot get lidar
    ROS_INFO("------------ START GET LIDAR %s! ------------", lidar_dir.c_str());
    rosbag::Bag in_bag;
    in_bag.open(lidar_dir, rosbag::bagmode::Read); 
     //创建view，用于读取bag中的topic
    std::vector<std::string> topics; 
    // topics.push_back(std::string("/livox/lidar"));   
    // topics.push_back(std::string("/hik_cam_node/hik_camera"));   
    // rosbag::View view_in_bag(in_bag, rosbag::TopicQuery(topics));
    
    rosbag::View view_in_bag(in_bag);
    for(rosbag::MessageInstance const msg: view_in_bag) 
    {
        livox_ros_driver::CustomMsgPtr livox_msg = msg.instantiate<livox_ros_driver::CustomMsg>();
        // sensor_msgs::PointCloud2ConstPtr livox_msg = msg.instantiate<sensor_msgs::PointCloud2>();
        
        if (livox_msg!= NULL)
        {
            for (int i = 0; i < livox_msg->point_num; ++i) {
                PointType pt;
                pt.x = livox_msg->points[i].x;
                pt.y = livox_msg->points[i].y;
                pt.z = livox_msg->points[i].z;
                pt.intensity = livox_msg->points[i].reflectivity;
                if (IS_FILTER && livox_msg->points[i].tag != 16)
                {
                    // 去除噪点（1. 基于能量判断的噪点 2. 基于空间位置判断的噪点）
                    continue;
                }
                if(pt.x == 0 && pt.y == 0 && pt.z == 0){continue;}
                pc_ptr_xyzi->push_back(pt);

            }
        }
        // sensor_msgs::ImageConstPtr msg_img = msg.instantiate<sensor_msgs::Image>();
        // if (msg_img!= NULL)
        // {
        //     ROS_INFO("topic = camera/image_raw");
        // }
    }
    ROS_INFO("lidar point cloud num = %d", pc_ptr_xyzi->points.size());
    in_bag.close();
}


int ImageLidarCalib::get_data()
{
    node_handle.getParam("lidar_dir", lidar_dir);
    node_handle.getParam("img_dir", img_dir);
    ROS_INFO("lidar_dir = %s", lidar_dir.c_str());
    // lidar_file_dir = save_dir + prefix + "_msg.bag";
    
    if (!IsFileExist(lidar_dir.c_str()) || !IsFileExist(img_dir.c_str()))
    {return -1;}
     // /////////////////////////////////////////////
    // get data and fusion
    // /////////////////////////////////////////////
    get_img();
    get_lidar();
    
    return 0;
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "calib_lidar_img_node");
  ImageLidarCalib img_lidar_calib_obj;
  int res = img_lidar_calib_obj.get_data();
  
  ROS_INFO("there are %d args", argc);
  
  
  return 0;
}
#include <unordered_map>


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
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>

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
#include "common.h"

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
    int res = get_data();
    std::unordered_map<VOXEL_LOC, Voxel *> voxel_map;
    // 对点云进行体素化，边长0.02m，Key是体素中心坐标(整数)，Value是体素颜色、体素坐标(浮点数)和具体点云
    initVoxel(this->pc_ptr_xyzi, 0.02, voxel_map);
    
    // // // 对激光雷达的边缘进行提取： plane_line_cloud_ 存储平面交接点云
    // LiDAREdgeExtraction(voxel_map, ransac_dis_threshold_, plane_size_threshold_,
    //                   plane_line_cloud_);

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

  // 将点云体素化
  void initVoxel(
    const PointCloudXYZI::Ptr &input_cloud,
    const float voxel_size, std::unordered_map<VOXEL_LOC, Voxel *> &voxel_map);

  void LiDAREdgeExtraction(
      const std::unordered_map<VOXEL_LOC, Voxel *> &voxel_map,
      const float ransac_dis_thre, const int plane_size_threshold,
      pcl::PointCloud<pcl::PointXYZI>::Ptr &lidar_line_cloud_3d);

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

void ImageLidarCalib::initVoxel(
    const PointCloudXYZI::Ptr &input_cloud,
    const float voxel_size, std::unordered_map<VOXEL_LOC, Voxel *> &voxel_map) {
  ROS_INFO_STREAM("Building Voxel");
  // for voxel test
  srand((unsigned)time(NULL));
  for (size_t i = 0; i < input_cloud->size(); i++) {
    // 体素化 （/oxel_size）
    const pcl::PointXYZI &p_c = input_cloud->points[i];
    float loc_xyz[3];
    for (int j = 0; j < 3; j++) {
      loc_xyz[j] = p_c.data[j] / voxel_size;
      if (loc_xyz[j] < 0) {
        loc_xyz[j] -= 1.0;
      }
    }
    VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
                       (int64_t)loc_xyz[2]);
    // voxel_map字典，从VOXEL_LOC -> Voxel
    if (voxel_map.find(position) != voxel_map.end()) {
      // 如果字典含值
      voxel_map[position]->cloud->push_back(p_c);
      
    } else {
      // 如果字典不含值
      Voxel *voxel = new Voxel(voxel_size);
      voxel_map[position] = voxel;
      voxel_map[position]->voxel_origin[0] = position.x * voxel_size;
      voxel_map[position]->voxel_origin[1] = position.y * voxel_size;
      voxel_map[position]->voxel_origin[2] = position.z * voxel_size;
      voxel_map[position]->cloud->push_back(p_c);
      int r = rand() % 256;
      int g = rand() % 256;
      int b = rand() % 256;
      voxel_map[position]->voxel_color << r, g, b;
    }
  }
  
  for (auto iter = voxel_map.begin(); iter != voxel_map.end(); iter++) {
    // iter->second对应value，iter->first 对应key
    if (iter->second->cloud->size() > 20) {
      // voxel_size = 0.02m, 类似PCL体素滤波，一个体素用一个点代替
      down_sampling_voxel(*(iter->second->cloud), 0.02);
    }
  }
  // // visualize
  ROS_INFO("----- start visualize Voxels -----");
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr test_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  for (auto iter = voxel_map.begin(); iter != voxel_map.end(); iter++) {
      pcl::PointXYZRGB p_rgb;
      p_rgb.x = iter->second->voxel_origin[0];
      p_rgb.y = iter->second->voxel_origin[1];
      p_rgb.z = iter->second->voxel_origin[2];
      p_rgb.r = iter->second->voxel_color(0);
      p_rgb.g = iter->second->voxel_color(1);
      p_rgb.b = iter->second->voxel_color(2);
      test_cloud->push_back(p_rgb);
  }
  ROS_INFO("test_cloud->size = %d", test_cloud->size());
  pcl::visualization::CloudViewer viewer("Simple Cloud Viewer"); //创造一个显示窗口
  viewer.showCloud(test_cloud);
  while (!viewer.wasStopped())
  {
  }

}

// void ImageLidarCalib::LiDAREdgeExtraction(
//     const std::unordered_map<VOXEL_LOC, Voxel *> &voxel_map,
//     const float ransac_dis_thre, const int plane_size_threshold,
//     pcl::PointCloud<pcl::PointXYZI>::Ptr &lidar_line_cloud_3d) {
//   ROS_INFO_STREAM("Extracting Lidar Edge");
//   ros::Rate loop(5000);
//   lidar_line_cloud_3d =
//       pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
//   for (auto iter = voxel_map.begin(); iter != voxel_map.end(); iter++) {
//       // 对0.02m x 0.02m x 0.02m的体素内点云进行操作
//     if (iter->second->cloud->size() > 50) 
//     {
//       // Plane 包括struct{平面中心、法向量、点云三部分}
//       std::vector<Plane> plane_list;
//       // 创建一个体素滤波器
//       pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filter(
//           new pcl::PointCloud<pcl::PointXYZI>);
//       pcl::copyPointCloud(*iter->second->cloud, *cloud_filter);
//       //创建一个模型参数对象，用于记录结果
//       pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
//       // inliers表示误差能容忍的点，记录点云序号
//       pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
//       //创建一个分割器
//       pcl::SACSegmentation<pcl::PointXYZI> seg;
//       // Optional,设置结果平面展示的点是分割掉的点还是分割剩下的点
//       seg.setOptimizeCoefficients(true);
//       // Mandatory-设置目标几何形状
//       seg.setModelType(pcl::SACMODEL_PLANE);
//       //分割方法：随机采样法
//       seg.setMethodType(pcl::SAC_RANSAC);
//       //设置误差容忍范围，也就是阈值
//       if (iter->second->voxel_origin[0] < 10) {
//         seg.setDistanceThreshold(ransac_dis_thre);
//       } else {
//         seg.setDistanceThreshold(ransac_dis_thre);
//       }
//       pcl::PointCloud<pcl::PointXYZRGB> color_planner_cloud;
//       int plane_index = 0;
//       // 对体素内的点云提取平面获得plane_lst
//       while (cloud_filter->points.size() > 10) 
//       {
//         pcl::PointCloud<pcl::PointXYZI> planner_cloud;
//         pcl::ExtractIndices<pcl::PointXYZI> extract;
//         //输入点云
//         seg.setInputCloud(cloud_filter);
//         seg.setMaxIterations(500);
//         //分割点云
//         seg.segment(*inliers, *coefficients);
//         if (inliers->indices.size() == 0) {
//           ROS_INFO_STREAM(
//               "Could not estimate a planner model for the given dataset");
//           break;
//         }
//         extract.setIndices(inliers);
//         extract.setInputCloud(cloud_filter);
//         extract.filter(planner_cloud);

//         if (planner_cloud.size() > plane_size_threshold) {
//           pcl::PointCloud<pcl::PointXYZRGB> color_cloud;
//           std::vector<unsigned int> colors;
//           colors.push_back(static_cast<unsigned int>(rand() % 256));
//           colors.push_back(static_cast<unsigned int>(rand() % 256));
//           colors.push_back(static_cast<unsigned int>(rand() % 256));
//           pcl::PointXYZ p_center(0, 0, 0);
//           for (size_t i = 0; i < planner_cloud.points.size(); i++) {
//             pcl::PointXYZRGB p;
//             p.x = planner_cloud.points[i].x;
//             p.y = planner_cloud.points[i].y;
//             p.z = planner_cloud.points[i].z;
//             p_center.x += p.x;
//             p_center.y += p.y;
//             p_center.z += p.z;
//             p.r = colors[0];
//             p.g = colors[1];
//             p.b = colors[2];
//             color_cloud.push_back(p);
//             color_planner_cloud.push_back(p);
//           }
//           p_center.x = p_center.x / planner_cloud.size();
//           p_center.y = p_center.y / planner_cloud.size();
//           p_center.z = p_center.z / planner_cloud.size();
//           Plane single_plane;
//           single_plane.cloud = planner_cloud;
//           single_plane.p_center = p_center;
//           single_plane.normal << coefficients->values[0],
//               coefficients->values[1], coefficients->values[2];
//           single_plane.index = plane_index;
//           plane_list.push_back(single_plane);
//           plane_index++;
//         }
//         extract.setNegative(true);
//         pcl::PointCloud<pcl::PointXYZI> cloud_f;
//         extract.filter(cloud_f);
//         *cloud_filter = cloud_f;
//       }
//       // 获得plane_lst后，计算边缘
//       if (plane_list.size() >= 2) {
//         sensor_msgs::PointCloud2 planner_cloud2;
//         pcl::toROSMsg(color_planner_cloud, planner_cloud2);
//         planner_cloud2.header.frame_id = "livox";
//         planner_cloud_pub_.publish(planner_cloud2);
//         loop.sleep();
//       }

//       std::vector<pcl::PointCloud<pcl::PointXYZI>> line_cloud_list;
//       calcLine(plane_list, voxel_size_, iter->second->voxel_origin,
//                line_cloud_list);
//       // ouster 5,normal 3 一个体素中最多8条边缘(0.02m x 0.02m x 0.02m)
//       if (line_cloud_list.size() > 0 && line_cloud_list.size() <= 8) {

//         for (size_t cloud_index = 0; cloud_index < line_cloud_list.size();
//              cloud_index++) {
//           for (size_t i = 0; i < line_cloud_list[cloud_index].size(); i++) {
//             pcl::PointXYZI p = line_cloud_list[cloud_index].points[i];
//             plane_line_cloud_->points.push_back(p);
//             sensor_msgs::PointCloud2 pub_cloud;
//             pcl::toROSMsg(line_cloud_list[cloud_index], pub_cloud);
//             pub_cloud.header.frame_id = "livox";
//             line_cloud_pub_.publish(pub_cloud);
//             loop.sleep();
//             plane_line_number_.push_back(line_number_);
//           }
//           line_number_++;
//         }
//       }
//     }
//   }
// }


int main(int argc, char **argv) {
  ros::init(argc, argv, "calib_lidar_img_node");
  ImageLidarCalib img_lidar_calib_obj;
  
  ROS_INFO("there are %d args", argc);
  
  
  return 0;
}
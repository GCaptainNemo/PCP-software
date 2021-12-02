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
#include "utils.h"

#include <Eigen/Dense>
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>


#define ROS_MIN_MAJOR 1
#define ROS_MIN_MINOR 8
#define ROS_MIN_PATCH 16

#if ROS_VERSION_MINIMUM(ROS_MIN_MAJOR, ROS_MIN_MINOR, ROS_MIN_PATCH)
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/CvBridge.h>
#endif

#define foreach BOOST_FOREACH
const bool IS_FILTER = true;
const bool IS_IMAGE_CORRECTION = true;

std::vector<cv::Point2f> corners(0);
std::vector<cv::Point2f> ir_corners(0);
std::vector<float> depth_vector(0);

cv_bridge::CvImagePtr cv_ptr;
cv::Mat out_img;
typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;

cv::Mat range_img;      /**< record z(unit m)*/


// PointCloudXYZI::Ptr pc_ptr_xyzi(new PointCloudXYZI); 
// PointCloudXYZRGB::Ptr pc_ptr_xyzrgb(new PointCloudXYZRGB);


class ImageLivoxFusion
{
public:
    cv::Mat transform_matrix;    /**< from globol to image coordinate */
    cv::Mat intrinsic_matrix;    /**< from local to image coordinate  */
	cv::Mat extrinsic_matrix;    /**< from global to local coordinate */
	cv::Mat dist_matrix;         /**< dist parameters  */
    int rgb_img_height;
    int rgb_img_width;
    int ir_img_height;
    int ir_img_width;
    
    cv::Mat rgb_img;
    cv::Mat color_range_img;
    cv::Mat ir_img;
    cv::Mat mask_img;       /**count reprojection times*/

public:
    PointCloudXYZI::Ptr pc_ptr_xyzi; 
    PointCloudXYZRGB::Ptr pc_ptr_xyzrgb;

public:
    ros::NodeHandle node_handle;
    std::string lidar_file_dir;
    std::string rgb_file_dir;
    std::string ir_file_dir;
    std::string save_dir;
    std::string prefix;
    
public:
  ImageLivoxFusion()
  {
    ROS_INFO("------------ intialize ----------\n");
    pc_ptr_xyzi.reset(new PointCloudXYZI());
    pc_ptr_xyzrgb.reset(new PointCloudXYZRGB());
    
    // livox_sub = node_handle.subscribe("/livox/lidar", 100, &ImageLivoxFusion::livoxCallback, this);
    
   
    this->set_param();
   
    
  };

   ~ImageLivoxFusion()
  {
  };
  // get camera parameters
  void set_param();

  // get lidar, infrared and visual image and fusion(get colored lidar)
  int get_data_and_fusion();

  // read visual image and infrared image 
  void get_imgs();

  // get lidar 
  void get_lidar();

  // fusion get colored pointcloud and range image
  void fusion();

  // choose image correspondance points
  void choose_corners();

  // use direct linear transform calculate projected matrix and project range image to infrared image
  void dlt();
  
};

void ImageLivoxFusion::dlt()
{
    if (corners.size() != ir_corners.size()){return;}
    Eigen::Matrix3d int_mat;
    cv::cv2eigen(this->intrinsic_matrix, int_mat);
    Eigen::Matrix3d inv_int_mat = int_mat.inverse();
    ROS_INFO_STREAM("int_mat = " << int_mat << "\n"); 
    std::vector<Eigen::Vector3d> world_pos_array(0);
    std::vector<Eigen::Vector2d> dst_pos_array(0);

    for(int i=0; i < corners.size(); ++i)
    {
        int x = corners[i].x;
        int y = corners[i].y;
        // double depth = (double)range_img.at<float>(y, x);
        double depth = depth_vector[i];

        ROS_INFO("depth = %f", depth);
        if(depth < 1e-3){continue;}
        Eigen::Vector3d camera_pos((double) x, (double) y, 1.0);
        Eigen::Vector3d world_pos = inv_int_mat * camera_pos;
        Eigen::Vector2d dst_pos((double) ir_corners[i].x, (double) ir_corners[i].y);
        world_pos = world_pos / world_pos(2) * depth;
        ROS_INFO_STREAM("world_pos = " << world_pos.transpose() << "\n");
        world_pos_array.push_back(world_pos);
        dst_pos_array.push_back(dst_pos);
    }
    Eigen::MatrixXd A(dst_pos_array.size() * 2, 11);
    Eigen::MatrixXd b(dst_pos_array.size() * 2, 1);
    Eigen::MatrixXd solve(11, 1);
    
    for(int i=0; i < dst_pos_array.size(); ++i)
    {
        double X = world_pos_array[i](0);
        double Y = world_pos_array[i](1);
        double Z = world_pos_array[i](2);
        double u = dst_pos_array[i](0);
        double v = dst_pos_array[i](1);
        A.row(2 * i) << X, Y, Z, 1.0, 0., 0., 0., -u*X , -u * Y , -u * Z;
        A.row(2 * i + 1) << 0., 0., 0., 0., X, Y, Z, 1.0, -v*X, -v * Y, -v * Z;
        // Eigen::MatrixXd block_up(1, 11);
        // block_up << X << Y << Z << 1.0 << 0.0 << 0.0 << 0.0 << 0.0 << -u*X << -u * Y << -u * Z;
        // Eigen::MatrixXd block_down(1, 11);
        // block_down << 0. << 0. << 0. << 0. << X << Y << Z << 1.0 << -v*X << -v * Y << -v * Z;
        // A.block(i * 2, 0, 1, 11) = block_up;
        // A.block(i * 2 + 1, 8, 1, 11) << block_down;
        b(i * 2, 0) = u;
        b(i * 2 + 1, 0) = v;
    }
    solve = A.colPivHouseholderQr().solve(b);
    ROS_INFO_STREAM("ANSWER = " << solve << "\n");
}

void ImageLivoxFusion::choose_corners()
{
    // ///////////////////////////////////////////////////////
    // choose corner
    // //////////////////////////////////////////////////////
    std::vector<cv::Mat> hImgs;
    hImgs.push_back(color_range_img);
    hImgs.push_back(rgb_img);
    hImgs.push_back(ir_img);

    cv::Mat cat_img;
    cv::hconcat(hImgs, cat_img);
    // cv::Mat clone_src_img = rgb_img.clone();
    IplImage *img = new IplImage(cat_img);
    cvNamedWindow("src", 1);
    cvSetMouseCallback("src", on_mouse, img);
    cvShowImage("src", img);
    
    char *obj_key = "q";
    while ((char)cv::waitKey(1) != *obj_key) {}
    // cvWaitKey(0);
    // cvDestroyAllWindows();
    delete img;
    // cvReleaseImage(&img);
    // corners-> std::vector<cv::Point>
    if (!corners.size()) {
        cout << "No input corners, end process" << endl;
        return ;
    }
    this->dlt();

};


int ImageLivoxFusion::get_data_and_fusion()
{
    node_handle.getParam("save_dir",save_dir);
    node_handle.getParam("prefix", prefix);
    ROS_INFO("save_dir = %s", save_dir.c_str());
    if(!IsFolderExist(save_dir.c_str())){ROS_ERROR("save dir doesn't exist"); return -1;}
    ROS_INFO("save dir exists!!");
    // lidar_file_dir = save_dir + prefix + "_msg.bag";
    lidar_file_dir = save_dir + prefix + "_lidar.bag";    
    rgb_file_dir = save_dir + prefix + "_rgb.jpg";
    ir_file_dir = save_dir + prefix + "_ir.jpg";
    
    
    if (!IsFileExist(lidar_file_dir.c_str()) || !IsFileExist(rgb_file_dir.c_str()))
    {return -1;}
     // /////////////////////////////////////////////
    // get data and fusion
    // /////////////////////////////////////////////
    get_imgs();
    get_lidar();
    fusion();
    return 0;
}

void ImageLivoxFusion::set_param() 
{
  // extrinsic matrix parameters
  XmlRpc::XmlRpcValue param_list;
  std::vector<double> Extrin_matrix;
  if(!node_handle.getParam("/review_color_lidar_node/CameraExtrinsicMat/data", param_list))
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
  if(!node_handle.getParam("/review_color_lidar_node/CameraMat/data", param_list))
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
  if(!node_handle.getParam("/review_color_lidar_node/DistCoeff/data", param_list))
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
  if(!node_handle.getParam("/review_color_lidar_node/ImageSize", param_list))
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
  rgb_img_width = img_size[0];
  rgb_img_height = img_size[1];
  range_img = cv::Mat::zeros(rgb_img_height, rgb_img_width, CV_32FC1);
  mask_img = cv::Mat::zeros(rgb_img_height, rgb_img_width, CV_8UC1);
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

void ImageLivoxFusion::get_imgs()
{
  this->rgb_img = cv::imread(rgb_file_dir, -1);
  if (IS_IMAGE_CORRECTION){
    cv::Mat undist_img;
    cv::undistort(rgb_img, undist_img, this->intrinsic_matrix, this->dist_matrix);
    rgb_img = undist_img.clone();
  }
  this->ir_img = cv::imread(ir_file_dir, -1);
  ir_img_height = ir_img.rows;
  ir_img_width = ir_img.cols;
  cv::Size dst_size(rgb_img_width, rgb_img_height);
  
  cv::resize(ir_img, ir_img, dst_size, cv::INTER_CUBIC);
//   cv::imshow("src", rgb_img);
//   cv::waitKey(0);
};

void ImageLivoxFusion::get_lidar()
{
    // cannot get lidar
    ROS_INFO("------------ START GET LIDAR %s! ------------", lidar_file_dir.c_str());
    rosbag::Bag in_bag;
    in_bag.open(lidar_file_dir, rosbag::bagmode::Read); 
     //创建view，用于读取bag中的topic
    std::vector<std::string> topics; 
    // topics.push_back(std::string("/livox/lidar"));   
    // topics.push_back(std::string("/hik_cam_node/hik_camera"));   
    // rosbag::View view_in_bag(in_bag, rosbag::TopicQuery(topics));
    
    rosbag::View view_in_bag(in_bag);
    for(rosbag::MessageInstance const msg: view_in_bag) 
    {
        livox_ros_driver::CustomMsgPtr livox_msg = msg.instantiate<livox_ros_driver::CustomMsg>();
        // sensor_msgs::PointCloud2ConstPtr s = msg.instantiate<sensor_msgs::PointCloud2>();
        // sensor_msgs::PointCloud2ConstPtr livox_msg = msg.instantiate<sensor_msgs::PointCloud2>();
        
        if (livox_msg!= NULL)
        {
            // ROS_INFO("topic = livox/lidar");
            for (int i = 0; i < livox_msg->point_num; ++i) {
                // PointXYZINormal
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

void ImageLivoxFusion::fusion()
{
    ROS_INFO("------------ START FUSION! ------------");
    int size = pc_ptr_xyzi->points.size();
    for (int i = 0; i < size; i++)
    {
        // project get the photo coordinate
        // pcl::PointXYZRGB pointRGB;
        pcl::PointXYZRGB pointRGB;

        pointRGB.x = pc_ptr_xyzi->points[i].x;
        pointRGB.y = pc_ptr_xyzi->points[i].y;
        pointRGB.z = pc_ptr_xyzi->points[i].z;
        double a_[4] = { pointRGB.x, pointRGB.y, pointRGB.z, 1.0 };
        cv::Mat pos(4, 1, CV_64F, a_);
        cv::Mat newpos(this->transform_matrix * pos);

        float x = (float)(newpos.at<double>(0, 0) / newpos.at<double>(2, 0));
        float y = (float)(newpos.at<double>(1, 0) / newpos.at<double>(2, 0));
        
        // ROS_INFO("x = %f, y =%f", x, y);
        // Trims viewport according to image size
        if (pointRGB.x >= 0)
        {
          if (x >= 0 && x < rgb_img_width && y >= 0 && y < rgb_img_height)
          {
            //  imread BGR（BITMAP）
            int row = int(y + 0.5);
            int column = int(x + 0.5);
            if (row < rgb_img_height && column < rgb_img_width)
            {
                // ROS_INFO("row = %d, column =%d", row, column);

                pointRGB.r = rgb_img.at<cv::Vec3b>(row, column)[2];
                pointRGB.g = rgb_img.at<cv::Vec3b>(row, column)[1];
                pointRGB.b = rgb_img.at<cv::Vec3b>(row, column)[0];
                // ROS_INFO("row = %d, column =%d", row, column);
                float val = range_img.at<float>(row, column);
                if (val == 0 || pointRGB.x < val)
                {
                    range_img.at<float>(row, column) = pointRGB.x;
                    mask_img.at<int>(row, column) += 1;
                }
                pc_ptr_xyzrgb->push_back(pointRGB);
            }
          }
        }
    }
    pc_ptr_xyzrgb->width = 1;
    pc_ptr_xyzrgb->height = pc_ptr_xyzrgb->points.size();
    ROS_INFO("pc_xyzrgb.size = %d", pc_ptr_xyzrgb->points.size());
    // range image processing and show
    double minVal = 0.0;
    double maxVal = 0.0;
    cv::minMaxLoc(range_img, &minVal, &maxVal);
    ROS_INFO_STREAM( "range image min, max = "<< minVal << ", " << maxVal);
    double min_val = 0;
    double max_val = 0;
    cv::minMaxLoc(mask_img, &min_val, &max_val);
    ROS_INFO_STREAM( "mask image min, max = "<< min_val << ", " << max_val);

    cv::Mat normalize_range_img;
    cv::normalize(range_img, normalize_range_img, 1.0, 0, cv::NORM_MINMAX);
    cv::Mat range_image_8uc1;
    
    normalize_range_img.convertTo(range_image_8uc1,CV_8UC1,255.0);
    ROS_INFO_STREAM("convertTo CV_8UC1");
	ROS_INFO_STREAM("ImgColorC3.type()" << range_image_8uc1.type());
    

    cv::applyColorMap(range_image_8uc1, color_range_img, cv::COLORMAP_JET);
    
    // cv::applyColorMap(mask_img, mask_im_color, cv::COLORMAP_JET);
    // cv::imshow("range_image", normalize_range_img);
    // cv::imshow("range_image", color_range_img);
    // cv::waitKey(0);
    // cv::imshow("mask_image", mask_im_color);
    // cv::waitKey(0);

}   



int main(int argc, char **argv) {
  ros::init(argc, argv, "review_color_lidar_node");
  ImageLivoxFusion img_lidar_fusion_obj;
  int res = img_lidar_fusion_obj.get_data_and_fusion();
  img_lidar_fusion_obj.choose_corners();
  // save
  if (res != 0){
      ROS_INFO("RES = %d", res);
      return -1;
  }
  ROS_INFO("there are %d args", argc);
  if (strcmp(argv[1], "true") == 0) 
  {
      ROS_INFO("START SAVING");
      std::string output_dir = img_lidar_fusion_obj.save_dir + img_lidar_fusion_obj.prefix + std::string(".pcd");
      pcl::io::savePCDFile(output_dir, *(img_lidar_fusion_obj.pc_ptr_xyzrgb));
  }
  // visualize
  if (strcmp(argv[2], "true") == 0) 
  {
    ROS_INFO("START visualizing");
    pcl::visualization::CloudViewer viewer("Simple Cloud Viewer"); //创造一个显示窗口
    viewer.showCloud(img_lidar_fusion_obj.pc_ptr_xyzrgb);
    while (!viewer.wasStopped())
    {
    }
  }
  
  return 0;
}


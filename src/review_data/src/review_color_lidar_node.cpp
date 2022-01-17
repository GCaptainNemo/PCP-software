#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
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
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <opencv/highgui.h>
#include <stdio.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>
#include "utils.h"
#include "io_utils.h"

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
typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;

constexpr bool IS_FILTER = true;
// constexpr bool IS_IMAGE_CORRECTION = true;
constexpr bool IS_IMAGE_CORRECTION = false;
constexpr bool IsVoxelFilter = true;


std::vector<cv::Point2f> corners(0);
std::vector<cv::Point2f> ir_corners(0);
std::vector<float> depth_vector(0);
std::vector<float> dense_depth_vector(0);

cv_bridge::CvImagePtr cv_ptr;
cv::Mat out_img;


cv::Mat range_img;      /**< record z(unit m)*/
cv::Mat dense_range_img;
cv::Mat color_range_img;
cv::Mat color_dense_range_img;
    

// PointCloudXYZI::Ptr pc_ptr_xyzi(new PointCloudXYZI); 
// PointCloudXYZRGB::Ptr pc_ptr_xyzrgb(new PointCloudXYZRGB);

void mapJet(double v, double vmin, double vmax, uint8_t &r, uint8_t &g,
            uint8_t &b) {
  r = 255;
  g = 255;
  b = 255;

  if (v < vmin) {
    v = vmin;
  }

  if (v > vmax) {
    v = vmax;
  }

  double dr, dg, db;

  if (v < 0.1242) {
    db = 0.504 + ((1. - 0.504) / 0.1242) * v;
    dg = dr = 0.;
  } else if (v < 0.3747) {
    db = 1.;
    dr = 0.;
    dg = (v - 0.1242) * (1. / (0.3747 - 0.1242));
  } else if (v < 0.6253) {
    db = (0.6253 - v) * (1. / (0.6253 - 0.3747));
    dg = 1.;
    dr = (v - 0.3747) * (1. / (0.6253 - 0.3747));
  } else if (v < 0.8758) {
    db = 0.;
    dr = 1.;
    dg = (0.8758 - v) * (1. / (0.8758 - 0.6253));
  } else {
    db = 0.;
    dg = 0.;
    dr = 1. - (v - 0.8758) * ((1. - 0.504) / (1. - 0.8758));
  }

  r = (uint8_t)(255 * dr);
  g = (uint8_t)(255 * dg);
  b = (uint8_t)(255 * db);
}

// ir-rgb match point
typedef struct PairPt
{
public:
  PairPt(const cv::Point2f &ir, const cv::Point2f &rgb): ir_pt(ir), rgb_pt(rgb){};
  PairPt(){};
public:
  cv::Point2f ir_pt;
  cv::Point2f rgb_pt;
} PairPt;

class IrRgbRegistration
{
public:
    // Camera Matrix
    // K: intrinsic matrix
    // T: extrinsic matrix [R|t]
    // P: projection matrix K[R|t]
    // D: distortion matrix 
    cv::Mat K_rgb;        
	  cv::Mat T_rgb2lidar;    	  
    cv::Mat P_rgb2lidar;    
    cv::Mat D_rgb;    
    
    cv::Mat K_ir;
    cv::Mat T_rgb2ir;
    cv::Mat T_ir2lidar;
    cv::Mat P_ir2lidar;
    cv::Mat D_ir;

public:
    int rgb_img_height;
    int rgb_img_width;
    int ir_img_height;
    int ir_img_width;
    
    cv::Mat rgb_img;
    cv::Mat ir_img;
    cv::Mat mask_img;       /**count reprojection times*/
    cv::Mat intensity_img;

public:
    PointCloudXYZI::Ptr pc_ptr_xyzi; 
    PointCloudXYZRGB::Ptr pc_ptr_xyzrgb;
    PointCloudXYZRGB::Ptr pc_ptr_xyzir;

public:
    std::vector <PairPt> pair_pts;

public:
    ros::NodeHandle node_handle;
    std::string lidar_file_dir;
    std::string rgb_file_dir;
    std::string ir_file_dir;
    std::string ir_rgb_stereo_dir;
    std::string save_dir;
    std::string prefix;
    
public:
  IrRgbRegistration()
  {
    ROS_INFO("------------ intialize ----------\n");
    node_handle.getParam("save_dir",this->save_dir);
    node_handle.getParam("prefix", this->prefix);
    ROS_INFO("save_dir = %s", save_dir.c_str());
    if(!IsFolderExist(save_dir.c_str())){ROS_ERROR("save dir doesn't exist"); throw -1;}
    ROS_INFO("save dir exists!!");
    // lidar_file_dir = save_dir + prefix + "_msg.bag";
    this->lidar_file_dir = save_dir + prefix + "_lidar.bag";    
    this->rgb_file_dir = save_dir + prefix + "_rgb.jpg";
    this->ir_file_dir = save_dir + prefix + "_ir.jpg";
    this->ir_rgb_stereo_dir = save_dir + prefix + "_stereocalib_ir.yaml";
    if (!IsFileExist(lidar_file_dir.c_str()) || !IsFileExist(rgb_file_dir.c_str()) || !IsFileExist(ir_file_dir.c_str()) || !IsFileExist(ir_rgb_stereo_dir.c_str()))
    {throw -1;}
    
    pc_ptr_xyzi.reset(new PointCloudXYZI());
    pc_ptr_xyzrgb.reset(new PointCloudXYZRGB());
    pc_ptr_xyzir.reset(new PointCloudXYZRGB());
    
    // livox_sub = node_handle.subscribe("/livox/lidar", 100, &IrRgbRegistration::livoxCallback, this);
    
   
    get_rgb_param();  // get rgb camera params
    get_ir_param();  // get ir camera params
    get_datas();    // get data frame
  
   
    
  };

   ~IrRgbRegistration()
  {
  };
  // get rgb camera parameters (K_rgb, D_rgb, T lidar->rgb)
  void get_rgb_param();

  // get ir camera parameters (K_ir, D_ir, T rgb->ir)
  void get_ir_param();

  // //////////////////////////////////////////////////////////////////////////
  // get lidar, infrared and visual image and fusion(get colored lidar)
  void get_datas();

  // read visual image and infrared image 
  void get_imgs();

  // get lidar 
  void get_lidar();
  // //////////////////////////////////////////////////////////////////////////
  // projected lidar pts(world coordinate) to rgb(K_rgb * T_rgb->lidar) and infrared image(K_ir * T_ir->lidar) to get matching points.
  void match_ir_rgb_pts();

  // fusion get colored pointcloud and range image
  void fusion_rgb_lidar();

  // fusion get ir colored pointclou
  void fusion_ir_lidar();

  // calculate Matching points
  void get_ir_rgb_match_pts();

  // draw match
  void draw_matches(const std::vector<PairPt> &pair_pts, const cv::Mat &ir_img, const cv::Mat &rgb_img);

  // choose image correspondance points
  void choose_corners();

  // use direct linear transform calculate projected matrix and project range image to infrared image
  void dlt();
  
};

void IrRgbRegistration::dlt()
{
    if (corners.size() != ir_corners.size()){return;}
    Eigen::Matrix3d int_mat;
    cv::cv2eigen(this->K_rgb, int_mat);
    Eigen::Matrix3d inv_int_mat = int_mat.inverse();
    ROS_INFO_STREAM("int_mat = " << int_mat << "\n"); 
    std::vector<Eigen::Vector3d> world_pos_array(0);
    std::vector<Eigen::Vector2d> dst_pos_array(0);

    for(int i=0; i < corners.size(); ++i)
    {
        int x = corners[i].x;
        int y = corners[i].y;
        // double depth = (double)range_img.at<float>(y, x);
        // double depth = depth_vector[i];
        double depth = dense_depth_vector[i];

        ROS_INFO("depth = %f", depth);
        if(depth < 1e-3){continue;}
        Eigen::Vector3d camera_pos((double) x, (double) y, 1.0);
        Eigen::Vector3d world_pos = inv_int_mat * camera_pos;
        world_pos = world_pos / world_pos(2) * depth;
        Eigen::Vector2d dst_pos((double) ir_corners[i].x, (double) ir_corners[i].y);
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
        A.row(2 * i) << X, Y, Z, 1.0, 0., 0., 0., 0., -u*X , -u * Y , -u * Z;
        A.row(2 * i + 1) << 0., 0., 0., 0., X, Y, Z, 1.0, -v*X, -v * Y, -v * Z;
        b(i * 2, 0) = u;
        b(i * 2 + 1, 0) = v;
    }
    solve = A.colPivHouseholderQr().solve(b);
    ROS_INFO_STREAM("ANSWER = " << solve << "\n");
    Eigen::MatrixXd project_mat(3, 4);
    project_mat << solve(0, 0), solve(1, 0), solve(2, 0), solve(3, 0), 
                   solve(4, 0), solve(5, 0), solve(6, 0), solve(7, 0), 
                   solve(8, 0), solve(9, 0), solve(10, 0), 1.0;
    project_mat << 766.84,  -336.343,  -118.382,   793.273, 
    251.936,   85.3765,  -79.0888,   579.974, 
    0.539377,  -0.21987, -0.161199, 1;
    ROS_INFO_STREAM("project_mat = " << project_mat << "\n");
    cv::Mat regist_rgb = cv::Mat::zeros(range_img.rows, range_img.cols, CV_8UC3);
    
    ROS_INFO("row = %d, col=%d", range_img.rows, range_img.cols);

    for(int row = 0; row < range_img.rows; ++row)
    {
        for(int col = 0; col < range_img.cols; ++col)
        {
            float depth = dense_range_img.at<float>(row, col);
            // float depth = range_img.at<float>(row, col);
            if (depth < 1e-3){continue;}
            Eigen::Vector3d camera_pos((double) col, (double) row, 1.0);
            Eigen::Vector3d world_pos = inv_int_mat * camera_pos;
            world_pos = world_pos / world_pos(2) * depth;   
            Eigen::VectorXd hetero_world_pos(4, 1);
            hetero_world_pos << world_pos, 1.0;
            
            Eigen::Vector3d proj_pos = project_mat * hetero_world_pos;
            double x = proj_pos(0) / proj_pos(2);
            double y = proj_pos(1) / proj_pos(2);
            if(x >= 0 && x < range_img.cols && y >= 0 && y < range_img.rows)
            {
                int proj_row = (int) (y + 0.5);
                int proj_col = (int) (x + 0.5);
                if(proj_row < range_img.rows && proj_col < range_img.cols)
                {
                    regist_rgb.at<cv::Vec3b>(proj_row, proj_col)[2] = rgb_img.at<cv::Vec3b>(row, col)[2];
                    regist_rgb.at<cv::Vec3b>(proj_row, proj_col)[1] = rgb_img.at<cv::Vec3b>(row, col)[1];
                    regist_rgb.at<cv::Vec3b>(proj_row, proj_col)[0] = rgb_img.at<cv::Vec3b>(row, col)[0];
                }
            }
        }
    }
    std::vector<cv::Mat> vImg;
    vImg.push_back(ir_img);
    vImg.push_back(regist_rgb);
    cv::Mat cat_img;
    cv::vconcat(vImg, cat_img);
    ROS_INFO("start show!");
    cv::imshow("regist_rgb", cat_img);
    cv::waitKey(0);

}

void IrRgbRegistration::choose_corners()
{
    std::string blur_type = std::string("gaussian");
    bool extrapolate = true;
    range_image_complete(range_img, dense_range_img, extrapolate, blur_type);
    // matwrite("raw_range_img.bin", range_img);
    // matread("raw_range_img.bin", range_img);
    cv::Mat normalize_dense_range_img;
    cv::normalize(dense_range_img, normalize_dense_range_img, 1.0, 0, cv::NORM_MINMAX);
    cv::Mat dense_range_image_8uc1;
    normalize_dense_range_img.convertTo(dense_range_image_8uc1, CV_8UC1,255.0);
    cv::applyColorMap(dense_range_image_8uc1, color_dense_range_img, cv::COLORMAP_JET);
    
    // ///////////////////////////////////////////////////////
    // choose corner
    // //////////////////////////////////////////////////////
    std::vector<cv::Mat> vImg1;
    std::vector<cv::Mat> vImg2;
    vImg1.push_back(rgb_img);
    vImg1.push_back(ir_img);
    // vImg2.push_back(color_range_img);
    cv::Mat map_img = cv::Mat::zeros(rgb_img.rows, rgb_img.cols, CV_8UC3);
    for (int x = 0; x < map_img.cols; x++) {
        for (int y = 0; y < map_img.rows; y++) {
        uint8_t r, g, b;
        float norm = intensity_img.at<uchar>(y, x) / 256.0;
        mapJet(norm, 0, 1, r, g, b);
        map_img.at<cv::Vec3b>(y, x)[0] = b;
        map_img.at<cv::Vec3b>(y, x)[1] = g;
        map_img.at<cv::Vec3b>(y, x)[2] = r;
        }
    }
    cv::Mat merge_img = 0.5 * map_img + 0.8 * rgb_img;
    vImg2.push_back(merge_img);
    
    vImg2.push_back(color_dense_range_img);
    cv::Mat cat_img_1;
    cv::Mat cat_img_2;
    cv::vconcat(vImg1, cat_img_1);
    cv::vconcat(vImg2, cat_img_2);
    std::vector<cv::Mat> hImg;
    hImg.push_back(cat_img_1);
    hImg.push_back(cat_img_2);
    cv::Mat cat_img;
    cv::hconcat(hImg, cat_img);

    // edited by why
    IplImage *img = new IplImage(cat_img);
    cvNamedWindow("src", 1);
    cvSetMouseCallback("src", on_mouse, img);
    cvShowImage("src", img);
    
    char *obj_key = "q";
    while ((char)cv::waitKey(1) != *obj_key) {}
    cvDestroyAllWindows();
    // delete img;
    // cvReleaseImage(&img);
    if (!corners.size()) {
        cout << "No input corners, end process" << endl;
        return ;
    }
    this->dlt();

};


void IrRgbRegistration::get_datas()
{
   
     // /////////////////////////////////////////////
    // get data and fusion
    // /////////////////////////////////////////////
    get_imgs();
    get_lidar();
}

void IrRgbRegistration::get_rgb_param() 
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
  this->rgb_img_width = img_size[0];
  this->rgb_img_height = img_size[1];
  range_img = cv::Mat::zeros(rgb_img_height, rgb_img_width, CV_32FC1);
  mask_img = cv::Mat::zeros(rgb_img_height, rgb_img_width, CV_8UC1);
  intensity_img = cv::Mat::zeros(rgb_img_height, rgb_img_width, CV_16UC1);
  // convert cv::Mat
  cv::Mat dist_array(5, 1, CV_64F, &dist[0]);
  this->D_rgb = dist_array.clone();
  
  
  this->K_rgb = cv::Mat(3, 3, CV_64F, &Intrinsic[0]);;

  
  cv::Mat ext_(4, 4, CV_64F, &Extrin_matrix[0]);
  cv::Mat invRt = ext_(cv::Rect(0, 0, 3, 3));
  cv::Mat R = invRt.t();
  cv::Mat invT = -R * ext_(cv::Rect(3, 0, 1, 3));
  cv::hconcat(R, invT, this->T_rgb2lidar);
  // transform matrix: from global coordinate to image coordinate
  this->P_rgb2lidar = this->K_rgb * this->T_rgb2lidar;
}

void IrRgbRegistration::get_ir_param()
{
  cv::FileStorage fs(ir_rgb_stereo_dir, cv::FileStorage::READ);
  fs["irCameraMat"] >>  this->K_ir;
  fs["irDistCoeff"] >>  this->D_ir;
  cv::Mat R_rgb2ir, t_rgb2ir;
  fs["R"] >> R_rgb2ir;
  fs["T"] >> t_rgb2ir;
  t_rgb2ir = t_rgb2ir / 1000;
  cv::hconcat(R_rgb2ir, t_rgb2ir, this->T_rgb2ir);
  cv::Mat R_rgb2lidar, t_rgb2lidar, R_ir2lidar, t_ir2lidar, R_ir2rgb, t_ir2rgb;
  R_rgb2lidar = this->T_rgb2lidar(cv::Rect(0, 0, 3, 3));  // x, y, width, height
  t_rgb2lidar = this->T_rgb2lidar(cv::Rect(3, 0, 1, 3));
  R_ir2rgb = R_rgb2ir.t();
  t_ir2rgb = - R_ir2rgb * t_rgb2ir;
  
  
  // //////////////////////////////////////////////
  R_ir2lidar = R_ir2rgb * R_rgb2lidar;  // infrared camera relative pos to lidar
  t_ir2lidar = R_ir2rgb * t_rgb2lidar + t_ir2rgb;
  cv::hconcat(R_ir2lidar, t_ir2lidar, this->T_ir2lidar);
  this->P_ir2lidar = this->K_ir * this->T_ir2lidar;
  ROS_INFO_STREAM("P_ir2lidar = " << P_ir2lidar << std::endl);

  // cv::vconcat(this->T_rgb2ir, )
}

void IrRgbRegistration::get_imgs()
{
  this->rgb_img = cv::imread(rgb_file_dir, -1);
  if (IS_IMAGE_CORRECTION){
    cv::Mat undist_img;
    cv::undistort(rgb_img, undist_img, this->K_rgb, this->D_rgb);
    rgb_img = undist_img.clone();
  }
  this->ir_img = cv::imread(ir_file_dir, -1);
  if (IS_IMAGE_CORRECTION){
    cv::Mat undist_img;
    cv::undistort(ir_img, undist_img, this->K_ir, this->D_ir);
    ir_img = undist_img.clone();
  }
  this->ir_img_height = ir_img.rows;
  this->ir_img_width = ir_img.cols;
  ROS_INFO("RGB_IMG (rows, cols, channels) = (%d, %d, %d)\n", rgb_img.rows, rgb_img.cols, rgb_img.channels());
  ROS_INFO("IR_IMG (rows, cols, channels) = (%d, %d, %d)\n", ir_img.rows, ir_img.cols, ir_img.channels());
  cv::imshow("ir_img", ir_img);
  cv::imshow("rgb_img", rgb_img);
  cv::waitKey(0);

  // cv::Size dst_size(rgb_img_width, rgb_img_height);
  
  // cv::resize(ir_img, ir_img, dst_size, cv::INTER_CUBIC);
//   cv::imshow("src", rgb_img);
//   cv::waitKey(0);
};

void IrRgbRegistration::get_lidar()
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



void IrRgbRegistration::fusion_rgb_lidar()
{
    ROS_INFO("------------ START FUSION rgb lidar! ------------");
    int size = pc_ptr_xyzi->points.size();
    for (int i = 0; i < size; i++)
    {
        // project get the photo coordinate
        // pcl::PointXYZRGB pointRGB;
        pcl::PointXYZRGB pointRGB;

        pointRGB.x = pc_ptr_xyzi->points[i].x;
        pointRGB.y = pc_ptr_xyzi->points[i].y;
        pointRGB.z = pc_ptr_xyzi->points[i].z;
        float intensity = pc_ptr_xyzi->points[i].intensity;
        double a_[4] = { pointRGB.x, pointRGB.y, pointRGB.z, 1.0 };
        cv::Mat pos(4, 1, CV_64F, a_);
        cv::Mat newpos(this->P_rgb2lidar * pos);

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
                // ROS_INFO("row = %d, column  =%d", row, column);
                
                pointRGB.r = rgb_img.at<cv::Vec3b>(row, column)[2];
                pointRGB.g = rgb_img.at<cv::Vec3b>(row, column)[1];
                pointRGB.b = rgb_img.at<cv::Vec3b>(row, column)[0];
                // ROS_INFO("row = %d, column =%d", row, column);
                float val = range_img.at<float>(row, column);
                if (val == 0 || pointRGB.x < val)
                {
                    range_img.at<float>(row, column) = pointRGB.x;
                    mask_img.at<int>(row, column) += 1;
                    if (intensity > 100) {
                        intensity = 65535;
                    } else {
                        intensity = (intensity / 150.0) * 65535;
                    }
                    intensity_img.at<uint16_t>(row, column) = (uint16_t) intensity;

                }
                pc_ptr_xyzrgb->push_back(pointRGB);
            }
          }
        }
    }
    intensity_img.convertTo(intensity_img, CV_8UC1, 1 / 256.0);
    ROS_INFO("finish pc_ptr");
    pc_ptr_xyzrgb->width = 1;
    pc_ptr_xyzrgb->height = pc_ptr_xyzrgb->points.size();
    ROS_INFO("pc_xyzrgb.size = %d", pc_ptr_xyzrgb->points.size());
    pcl::visualization::CloudViewer viewer("RGB CLOUD Viewer"); //创造一个显示窗口
    viewer.showCloud(this->pc_ptr_xyzrgb);
    while (!viewer.wasStopped())
    {
    }
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


void IrRgbRegistration::fusion_ir_lidar()
{
    ROS_INFO("------------ START FUSION infrared lidar! ------------");
    int size = pc_ptr_xyzi->points.size();
    for (int i = 0; i < size; i++)
    {
        // project get the photo coordinate
        // pcl::PointXYZRGB pointRGB;
        pcl::PointXYZRGB pointRGB;

        pointRGB.x = pc_ptr_xyzi->points[i].x;
        pointRGB.y = pc_ptr_xyzi->points[i].y;
        pointRGB.z = pc_ptr_xyzi->points[i].z;
        float intensity = pc_ptr_xyzi->points[i].intensity;
        double a_[4] = { pointRGB.x, pointRGB.y, pointRGB.z, 1.0 };
        cv::Mat pos(4, 1, CV_64F, a_);
        cv::Mat newpos(this->P_ir2lidar * pos);

        float x = (float)(newpos.at<double>(0, 0) / newpos.at<double>(2, 0));
        float y = (float)(newpos.at<double>(1, 0) / newpos.at<double>(2, 0));
        
        // ROS_INFO("x = %f, y =%f", x, y);
        // Trims viewport according to image size
        if (pointRGB.x >= 0)
        {
          if (x >= 0 && x < this->ir_img_width && y >= 0 && y < this->ir_img_height)
          {
            //  imread BGR（BITMAP）
            int row = int(y + 0.5);
            int column = int(x + 0.5);
            if (row < ir_img_height && column < ir_img_width)
            {
                // ROS_INFO("row = %d, column  =%d", row, column);
                
                pointRGB.r = ir_img.at<cv::Vec3b>(row, column)[2];
                pointRGB.g = ir_img.at<cv::Vec3b>(row, column)[1];
                pointRGB.b = ir_img.at<cv::Vec3b>(row, column)[0];
                
                pc_ptr_xyzir->push_back(pointRGB);
            }
          }
        }
    }
    
    pc_ptr_xyzir->width = 1;
    pc_ptr_xyzir->height = pc_ptr_xyzir->points.size();
    ROS_INFO("pc_ptr_xyzir.size = %d", pc_ptr_xyzir->points.size());
    // range image processing and show
    pcl::visualization::CloudViewer viewer("Simple Cloud Viewer"); //创造一个显示窗口
    viewer.showCloud(this->pc_ptr_xyzir);
    while (!viewer.wasStopped())
    {
    }
    
    // cv::applyColorMap(mask_img, mask_im_color, cv::COLORMAP_JET);
    // cv::imshow("range_image", normalize_range_img);
    // cv::imshow("range_image", color_range_img);
    // cv::waitKey(0);
    // cv::imshow("mask_image", mask_im_color);
    // cv::waitKey(0);

}   

void IrRgbRegistration::get_ir_rgb_match_pts()
{
  ROS_INFO("------------ START FUSION infrared lidar! ------------");
  pcl::PointCloud<pcl::PointXYZI> down_sample_pc_xyzi; // pts
  if (IsVoxelFilter) {
      // 体素滤波
      pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter; 
      downSizeFilter.setInputCloud(this->pc_ptr_xyzi); // ptr
      downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
      downSizeFilter.filter(down_sample_pc_xyzi);
  }
  else{
    down_sample_pc_xyzi = *this->pc_ptr_xyzi;
  }
  int size = down_sample_pc_xyzi.points.size();
  pair_pts.resize(0);
  
  for (int i = 0; i < size; i++)
  {
      // project get the photo coordinate
      // pcl::PointXYZRGB pointRGB;
      pcl::PointXYZI point_xyzi = down_sample_pc_xyzi.points[i];

      double a_[4] = { point_xyzi.x, point_xyzi.y, point_xyzi.z, 1.0 };
      cv::Mat pos(4, 1, CV_64F, a_);
      cv::Mat ir_pos(this->P_ir2lidar * pos);
      cv::Mat rgb_pos(this->P_rgb2lidar * pos);


      float ir_x = (float)(ir_pos.at<double>(0, 0) / ir_pos.at<double>(2, 0));
      float ir_y = (float)(ir_pos.at<double>(1, 0) / ir_pos.at<double>(2, 0));
      
      float rgb_x = (float)(rgb_pos.at<double>(0, 0) / rgb_pos.at<double>(2, 0));
      float rgb_y = (float)(rgb_pos.at<double>(1, 0) / rgb_pos.at<double>(2, 0));
      
      // ROS_INFO("x = %f, y =%f", x, y);
      // Trims viewport according to image size
      if (point_xyzi.x >= 0)
      {
        if (ir_x >= 0 && ir_x <= this->ir_img_width - 1 && ir_y >= 0 && ir_y <= this->ir_img_height - 1 && 
            rgb_x >= 0 && rgb_x <= this->rgb_img_width - 1 && rgb_y >= 0 && rgb_y <= this->rgb_img_height - 1)
        {
          PairPt pair_pt(cv::Point2f(ir_x, ir_y), cv::Point2f(rgb_x, rgb_y));
          pair_pts.push_back(pair_pt);
        }
      }
  }
  ROS_INFO("there are %d match pts\n", pair_pts.size());
} 
    
void IrRgbRegistration::draw_matches(const std::vector<PairPt> &pair_pts, const cv::Mat &ir_img, const cv::Mat &rgb_img)
{
  std::vector<cv::KeyPoint> rgb_key_pts, ir_key_pts;
  std::vector<cv::DMatch> matches;
  int index = 0;
  for(auto pair_pt: pair_pts)
  {
    cv::KeyPoint rgb_pt(pair_pt.rgb_pt, 1.f);
    cv::KeyPoint ir_pt(pair_pt.ir_pt, 1.f);
    rgb_key_pts.push_back(rgb_pt);
    ir_key_pts.push_back(ir_pt);
    if (index % 200 == 0) matches.push_back(cv::DMatch(index, index ,0.0));
    index++;
  }
  cv::Mat match_img;
  cv::drawMatches(rgb_img, rgb_key_pts, ir_img, ir_key_pts, matches, match_img, cv::Scalar(0, 255, 0));
  cv::imshow("match_img", match_img);
  cv::waitKey(0);



};





int main(int argc, char **argv) {
  ros::init(argc, argv, "review_color_lidar_node");
  IrRgbRegistration registration_obj;
  // #define DEBUG_POSE
  #ifdef DEBUG_POSE
  std::cout << "P_rgb2lidar = " << registration_obj.P_rgb2lidar << std::endl;
  std::cout << "P_ir2lidar = " << registration_obj.P_ir2lidar << std::endl;
  std::cout << "T_rgb2lidar = " << registration_obj.T_rgb2lidar << std::endl;
  std::cout << "T_ir2lidar = " << registration_obj.T_ir2lidar << std::endl;
  
  #endif
  // registration_obj.fusion_ir_lidar();
  // registration_obj.fusion_rgb_lidar();

  // registration_obj.choose_corners();
  registration_obj.get_ir_rgb_match_pts();
  registration_obj.draw_matches(registration_obj.pair_pts, registration_obj.ir_img , registration_obj.rgb_img);

  
  ROS_INFO("there are %d args", argc);
  if (strcmp(argv[1], "true") == 0) 
  {
      ROS_INFO("START SAVING");
      std::string output_dir = registration_obj.save_dir + registration_obj.prefix + std::string(".pcd");
      pcl::io::savePCDFile(output_dir, *(registration_obj.pc_ptr_xyzrgb));
  }
  // visualize
  if (strcmp(argv[2], "true") == 0) 
  {
    ROS_INFO("START visualizing");
    pcl::visualization::CloudViewer viewer("Simple Cloud Viewer"); //创造一个显示窗口
    viewer.showCloud(registration_obj.pc_ptr_xyzrgb);
    while (!viewer.wasStopped())
    {
    }
  }
  
  return 0;
}


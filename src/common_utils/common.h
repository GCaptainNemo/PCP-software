#ifndef LIDAR_CAMERA_COMMON_H
#define LIDAR_CAMERA_COMMON_H

#include <Eigen/Core>
#include <cv_bridge/cv_bridge.h>
#include <pcl/common/io.h>
#include <stdio.h>
#include <string>
#include <unordered_map>
typedef struct Voxel {
  float size;
  Eigen::Vector3d voxel_origin;
  Eigen::Vector3d voxel_color;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
  Voxel(float _size) : size(_size) {
    voxel_origin << 0, 0, 0;
    cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(
        new pcl::PointCloud<pcl::PointXYZI>);
  };
} Voxel;


class VOXEL_LOC {
public:
  int64_t x, y, z;

  VOXEL_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0)
      : x(vx), y(vy), z(vz) {}

  bool operator==(const VOXEL_LOC &other) const {
    return (x == other.x && y == other.y && z == other.z);
  }
};


struct M_POINT {
  float xyz[3];
  float intensity;
  int count = 0;
};

// 平面
typedef struct Plane {
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZ p_center;
  Eigen::Vector3d normal;
  int index;
} Plane;

// Hash value
namespace std {
template <> struct hash<VOXEL_LOC> {
  size_t operator()(const VOXEL_LOC &s) const {
    using std::size_t;
    using std::hash;
    return ((hash<int64_t>()(s.x) ^ (hash<int64_t>()(s.y) << 1)) >> 1) ^
           (hash<int64_t>()(s.z) << 1);
  }
};
}

// Similar with PCL voxelgrid filter
void down_sampling_voxel(pcl::PointCloud<pcl::PointXYZI> &pl_feat,
                         double voxel_size) {
  int intensity = rand() % 255;
  if (voxel_size < 0.01) {
    return;
  }
  std::unordered_map<VOXEL_LOC, M_POINT> feat_map;
  uint plsize = pl_feat.size();

  for (uint i = 0; i < plsize; i++) {
    pcl::PointXYZI &p_c = pl_feat[i];
    float loc_xyz[3];
    for (int j = 0; j < 3; j++) {
      loc_xyz[j] = p_c.data[j] / voxel_size;
      if (loc_xyz[j] < 0) {
        loc_xyz[j] -= 1.0;
      }
    }

    VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
                       (int64_t)loc_xyz[2]);
    auto iter = feat_map.find(position);
    if (iter != feat_map.end()) {
      iter->second.xyz[0] += p_c.x;
      iter->second.xyz[1] += p_c.y;
      iter->second.xyz[2] += p_c.z;
      iter->second.intensity += p_c.intensity;
      iter->second.count++;
    } else {
      M_POINT anp;
      anp.xyz[0] = p_c.x;
      anp.xyz[1] = p_c.y;
      anp.xyz[2] = p_c.z;
      anp.intensity = p_c.intensity;
      anp.count = 1;
      feat_map[position] = anp;
    }
  }
  plsize = feat_map.size();
  pl_feat.clear();
  pl_feat.resize(plsize);

  uint i = 0;
  for (auto iter = feat_map.begin(); iter != feat_map.end(); ++iter) {
    pl_feat[i].x = iter->second.xyz[0] / iter->second.count;
    pl_feat[i].y = iter->second.xyz[1] / iter->second.count;
    pl_feat[i].z = iter->second.xyz[2] / iter->second.count;
    pl_feat[i].intensity = iter->second.intensity / iter->second.count;
    i++;
  }
}

#endif

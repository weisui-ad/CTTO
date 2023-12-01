#include "common_utility.hpp"

/**
 * 对frame中的点放入grid中，将非空的voxel中的第一个点取出,实现降采样
*/
void subSampleFrame0(std::vector<point3D> &frame, double size_voxel){

     // mapping voxel<--> std::vector<point3D>
     std::tr1::unordered_map<voxel, std::vector<point3D>, std::hash<voxel>> grid;
     
     // 计算每个点处于哪个voxel，并将3d点添加到对应的voxel中
     for (int i = 0; i < (int)frame.size(); i++){
          auto kx = static_cast<short>(frame[i].raw_point[0] / size_voxel);
          auto ky = static_cast<short>(frame[i].raw_point[1] / size_voxel);
          auto kz = static_cast<short>(frame[i].raw_point[2] / size_voxel);
          grid[voxel(kx, ky, kz)].push_back(frame[i]);
     }

     // 将frame重置为0
     frame.resize(0);
     int step = 0;
     // 对于每一个非空的voxel, 返回第一个点
     for (const auto &n : grid){
          if (n.second.size() > 0){
               frame.push_back(n.second[0]);
               step++;
          }
     }
}

/**
 * 对frame中的点进行降采样，按照size_voxel_subsampling降采样率，将降采样之后的点存储到keypoints中
*/
void gridSampling0(const pcl::PointCloud<pcl::PointXYZI>::Ptr &frame,
                pcl::PointCloud<pcl::PointXYZI>::Ptr &keypoints, double size_voxel_subsampling){
     
     // 重置关键点
     keypoints.reset(new pcl::PointCloud<pcl::PointXYZI>());
     // frame_sub存储降采样之后的点
     std::vector<point3D> frame_sub;
     frame_sub.resize(frame->size());
     for (int i = 0; i < (int)frame_sub.size(); i++){
          point3D pt;
          pt.raw_point = Eigen::Vector3d(frame->points[i].x, frame->points[i].y, frame->points[i].z);
          // pt.point = pt.raw_point;
          pt.intensity = frame->points[i].intensity;
          frame_sub[i] = pt;
     }
     // std::cout << "input :" << frame->size() << ", " << frame_sub.size() << ",vs:" << size_voxel_subsampling << std::endl;
     // 对frame进行降采样，降采样之后的点存储到frame_sub
     subSampleFrame0(frame_sub, size_voxel_subsampling);
     // keypoints.reserve(frame_sub.size());
     for (int i = 0; i < (int)frame_sub.size(); i++){
          pcl::PointXYZI pt;
          pt.x = frame_sub[i].raw_point[0], pt.y = frame_sub[i].raw_point[1], pt.z = frame_sub[i].raw_point[2];
          pt.intensity = frame_sub[i].intensity;
          keypoints->push_back(pt);
     }
}
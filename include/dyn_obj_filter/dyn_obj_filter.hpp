#ifndef _DYN_OBJ_FILTER_H_
#define _DYN_OBJ_FILTER_H_

#include <iostream>
#include <functional>
#include <vector>
#include <deque>
#include <execution>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <Eigen/Core>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>

#include "utils.h"
#include "common_lib.h"
#include "dyn_obj_filter/filter_utils.hpp"

namespace dof {
template <typename PointT=PointType>
class DynamicObjectFilter {
public:
    DynamicObjectFilter() = default;
    ~DynamicObjectFilter() = default;
    void Init(ros::NodeHandle &_nh);
    void Process(const pcl::PointCloud<PointT> &input_pc, 
                 const M3D curr_rot, 
                 const V3D curr_pos, 
                 pcl::PointCloud<PointT> &static_pc, 
                 pcl::PointCloud<PointT> &dynamic_pc);
    void UpdateFilter(const pcl::PointCloud<PointT> &curr_static_points, const M3D curr_rot, const V3D curr_pos);

private:
    void LoadParams();
    void Reset(const pcl::PointCloud<PointT> &input_pc);

    /// detection
    void RangeImageProjection();
    void RangeImageProjection(const pcl::PointCloud<PointT> &input_pc);
    
    /// create voxel map for checking and clustering
    void CreateVoxelMap();

    /// check
    void FiltCandidateDynPoints();

    /// clustering
    void ExtractCluster ();
    void ExtendNearBlock(int idx);
    void ExtractDynamicCluster();

private:
    /// ros
    ros::NodeHandle nh;
    ros::Publisher pub_curr_range_img;
    ros::Publisher pub_hist_range_img;
    ros::Publisher pub_diff_range_img;

    /// flag
    bool first_flag = true;
    bool pub_range_img = false;
    bool use_check = false;
    bool use_cluster = false;

    /// point cloud
    std::deque<pcl::PointCloud<PointT>> hist_pc;
    std::deque<M3D> hist_rot;
    std::deque<V3D> hist_pos;
    pcl::PointCloud<PointT> curr_pc;
    int curr_pc_num;
    
    /// range image parameters
    double FOV_V_UP;
    double FOV_V_DOWN;
    double FOV_V;
    double FOV_H;
    double image_res;
    int img_rows;
    int img_cols;
    
    /// range image
    cv::Mat hist_range_img;
    cv::Mat curr_range_img;
    cv::Mat curr_count_img;
    cv::Mat curr_index_img;
    cv::Mat diff_range_img;
    
    /// detection parameters
    int window_size;
    float adaptive_coeff;
    float adaptive_diff;
    float dyn_ratio;
    int dyn_threshold;
    /// threshold
    int dyn_num_threshold;
    float dyn_ratio_threshold;
    
    /// check parameters
    double search_radius;
    
    /// hash map parameters
    float revolusion;
    float edge;
    
    /// clustering parameters
    int extend_block;
    float extend_range;
    
    VoxelHashMap voxel_hash_map;
    
    /// global
    std::vector<int> idx_lable;     // 0 for static, 1 for candidate dynamic, 2 for centainly dynamic
    std::vector<int> idx_vec;
    std::vector<PointType> pt_vec;
    std::vector<bool> processed_vec;
    std::vector<int> candidate_dyn_cluster;
};

} // namespace dof

#endif
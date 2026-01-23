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
#include <pcl/common/transforms.h> 

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
template <typename PointT>
void DynamicObjectFilter<PointT>::Init(ros::NodeHandle &_nh) {
    curr_pc.clear();
    
    nh = _nh;
    LoadParams();

    pub_curr_range_img = nh.advertise<sensor_msgs::Image>("/curr_range_img", 100000);
    pub_hist_range_img = nh.advertise<sensor_msgs::Image>("/hist_range_img", 100000);
    pub_diff_range_img = nh.advertise<sensor_msgs::Image>("/diff_range_img", 100000);
}

template <typename PointT>
void DynamicObjectFilter<PointT>::LoadParams() {
    /// flag
    nh.param<bool>("dof/flag/pub_range_img", pub_range_img, false);
    nh.param<bool>("dof/flag/use_check", use_check, false);
    nh.param<bool>("dof/flag/use_cluster", use_cluster, false);
    
    /// detection parameters
    nh.param<double>("dof/detection/FOV_V_UP", FOV_V_UP, 52);
    nh.param<double>("dof/detection/FOV_V_DOWN", FOV_V_DOWN, -7);
    FOV_V = std::abs(FOV_V_UP) + std::abs(FOV_V_DOWN);
    nh.param<double>("dof/detection/FOV_H", FOV_H, 360);

    nh.param<double>("dof/detection/image_res", image_res, 1.0);
    img_rows = std::round(FOV_V * image_res);
    img_cols = std::round(FOV_H * image_res);
    
    nh.param<int>("dof/detection/window_size", window_size, 5);
    nh.param<float>("dof/detection/adaptive_coeff", adaptive_coeff, 0.01);
    nh.param<float>("dof/detection/adaptive_diff", adaptive_diff, 0.03);
    nh.param<float>("dof/detection/dyn_ratio", dyn_ratio, 0.6);

    /// check parameters
    nh.param<double>("dof/check/search_radius", search_radius, 0.1);

    /// clustering parameters
    nh.param<float>("dof/cluster/revolusion", revolusion, 0.3);
    nh.param<float>("preprocess/max_range", edge, 100.0);

    nh.param<int>("dof/cluster/extend_block", extend_block, 1);
    nh.param<float>("dof/cluster/extend_range", extend_range, 0.1);
    
    nh.param<int>("dof/cluster/dyn_num_threshold", dyn_num_threshold, 10);
    nh.param<float>("dof/cluster/dyn_ratio_threshold", dyn_ratio_threshold, 0.5);
}

template <typename PointT>
void DynamicObjectFilter<PointT>::Reset(const pcl::PointCloud<PointT> &input_pc) {
    curr_pc.clear();
    curr_pc = input_pc;
    curr_pc_num = curr_pc.points.size();

    idx_lable.resize(curr_pc_num, 0);
    pt_vec.resize(curr_pc_num);
    processed_vec.resize(curr_pc_num, 0);
    for (int i = 0; i < curr_pc_num ; i++) {
        idx_lable[i] = 0;
        pt_vec[i] = input_pc.points[i];
        processed_vec[i] = 0;
    }

    voxel_hash_map.clear();
}

template <typename PointT>
void DynamicObjectFilter<PointT>::RangeImageProjection() {
    for (int idx = 0; idx < curr_pc_num; idx++) {
        const float x = curr_pc.points[idx].x;
        const float y = curr_pc.points[idx].y;
        const float z = curr_pc.points[idx].z;
        const float pitch = std::atan2(z, std::sqrt(x * x + y * y));
        const float yaw = std::atan2(y, x);
        const float range = std::sqrt(x * x + y * y + z * z);

        const float row = img_rows * (1.0f - (pitch * 180.0f / M_PI - FOV_V_DOWN) / FOV_V);
        const float col = img_cols * ((yaw * 180.0f / M_PI + FOV_H / 2.0f) / FOV_H);
        int v = static_cast<int>(std::round(row));
        int u = static_cast<int>(std::round(col));
        v = v > 0 ? v : 0;
        v = v < (img_rows - 1) ? v : (img_rows - 1);
        u = u > 0 ? u : 0;
        u = u < (img_cols - 1) ? u : (img_cols - 1);
        
        if (range < curr_range_img.at<float>(v, u)) {
            curr_range_img.at<float>(v, u) = range;
            curr_index_img.at<int>(v, u) = idx;
        }
    }
}

template <typename PointT>
void DynamicObjectFilter<PointT>::RangeImageProjection(const pcl::PointCloud<PointT> &input_pc) {
    for (PointType pt : input_pc.points) {
        const float x = pt.x;
        const float y = pt.y;
        const float z = pt.z;
        const float pitch = std::atan2(z, std::sqrt(x * x + y * y));
        const float yaw = std::atan2(y, x);
        const float range = std::sqrt(x * x + y * y + z * z);

        const float row = img_rows * (1.0f - (pitch * 180.0f / M_PI - FOV_V_DOWN) / FOV_V);
        const float col = img_cols * ((yaw * 180.0f / M_PI + FOV_H / 2.0f) / FOV_H);
        int v = static_cast<int>(std::round(row));
        int u = static_cast<int>(std::round(col));
        v = v > 0 ? v : 0;
        v = v < (img_rows - 1) ? v : (img_rows - 1);
        u = u > 0 ? u : 0;
        u = u < (img_cols - 1) ? u : (img_cols - 1);

        if (range < hist_range_img.at<float>(v, u)) {
            hist_range_img.at<float>(v, u) = range;
        }
    }
}

template <typename PointT>
void DynamicObjectFilter<PointT>::CreateVoxelMap() {
    for (int idx = 0; idx < curr_pc_num ; idx++) {
        PointType pt = pt_vec[idx];

        int vx = static_cast<int>(pt.x + edge);
        int vy = static_cast<int>(pt.y + edge);
        int vz = static_cast<int>(pt.z + edge);
        Voxel voxel(vx, vy, vz, revolusion);
        
        VoxelHashMap::iterator search = voxel_hash_map.find(voxel);
        if (search != voxel_hash_map.end()) {
            VoxelBlock &voxel_block = search.value();
            voxel_block.AddPoint(idx, pt, idx_lable[idx]);
        } else {
            VoxelBlock voxel_block;
            voxel_block.AddPoint(idx, pt, idx_lable[idx]);
            voxel_hash_map[voxel] = std::move(voxel_block);
        }
    }
}

template <typename PointT>
void DynamicObjectFilter<PointT>::FiltCandidateDynPoints() {
    for (int idx = 0; idx < curr_pc_num; idx++) {
        if (idx_lable[idx] == 1) {
            idx_lable[idx] = 0;
            PointType pt = pt_vec[idx];
            bool find_neighbor = false;
            for (int x_near_block = -extend_block; x_near_block <= extend_block; x_near_block++) {
                for (int y_near_block = -extend_block; y_near_block <= extend_block; y_near_block++) {
                    for (int z_near_block = -extend_block; z_near_block <= extend_block; z_near_block++) {
                        int vx = static_cast<int>(pt.x + edge);
                        int vy = static_cast<int>(pt.y + edge);
                        int vz = static_cast<int>(pt.z + edge);
                        Voxel voxel(vx, vy, vz, revolusion, x_near_block, y_near_block, z_near_block);
                        
                        VoxelHashMap::iterator search = voxel_hash_map.find(voxel);
                        if (search != voxel_hash_map.end()) {
                            VoxelBlock &voxel_block = search.value();
                            if (voxel_block.dyn_points_num != 0) {
                                find_neighbor = voxel_block.SearchDynNeighbor(pt, idx, idx_lable, search_radius);
                                if (find_neighbor) break;
                            }
                        }
                    }
                }
            }
        }
    }
}

template <typename PointT>
void DynamicObjectFilter<PointT>::ExtractCluster() {
    for (int idx = 0; idx < curr_pc_num; idx++) {
        if (!processed_vec[idx] && idx_lable[idx]) {
            ExtendNearBlock(idx);
            ExtractDynamicCluster();

            candidate_dyn_cluster.clear();
        }
    }
}

template <typename PointT>
void DynamicObjectFilter<PointT>::ExtendNearBlock(int idx) {
    candidate_dyn_cluster.emplace_back(idx);
    processed_vec[idx] = 1;
    PointType pt = pt_vec[idx];

    // region extend
    for (int x_near_block = -extend_block; x_near_block <= extend_block; x_near_block++) {
        for (int y_near_block = -extend_block; y_near_block <= extend_block; y_near_block++) {
            for (int z_near_block = -extend_block; z_near_block <= extend_block; z_near_block++) {
                int vx = static_cast<int>(pt.x + edge);
                int vy = static_cast<int>(pt.y + edge);
                int vz = static_cast<int>(pt.z + edge);
                Voxel voxel(vx, vy, vz, revolusion, x_near_block, y_near_block, z_near_block);
                
                VoxelHashMap::iterator search = voxel_hash_map.find(voxel);
                if (search != voxel_hash_map.end()) {
                    VoxelBlock &voxel_block = search.value();
                    for (int i = 0; i < voxel_block.points_num; i++) {
                        if (!processed_vec[voxel_block.idx_pt[i]]) {
                            double distance2 = CalculateDistance2(pt, voxel_block.pts[i]);
                            if (distance2 < extend_range * extend_range) {
                                candidate_dyn_cluster.emplace_back(voxel_block.idx_pt[i]);
                                ExtendNearBlock(voxel_block.idx_pt[i]);
                            }
                        }
                    }
                }
            }
        }
    }
}

template <typename PointT>
void DynamicObjectFilter<PointT>::ExtractDynamicCluster() {
    int dynamic_num = 0;
    int cluster_pt_num = candidate_dyn_cluster.size();
    
    for (int idx = 0; idx < cluster_pt_num; idx++) {
        if (idx_lable[candidate_dyn_cluster[idx]]) {
            dynamic_num++;
        } 
    }
        
    float dyn_ratio = static_cast<float>(dynamic_num) / cluster_pt_num;
    if (dynamic_num < dyn_num_threshold || dyn_ratio < dyn_ratio_threshold) {
        // static cluster
        for (int i = 0; i < cluster_pt_num; i++) {
            idx_lable[candidate_dyn_cluster[i]] = 0;
        }
    } else {
        // dynamic cluster
        for (int i = 0; i < cluster_pt_num; i++) {
            idx_lable[candidate_dyn_cluster[i]] = 2;
        }
    }
}

template <typename PointT>
void DynamicObjectFilter<PointT>::Process(const pcl::PointCloud<PointT> &input_pc, 
                                          const M3D curr_rot, 
                                          const V3D curr_pos, 
                                          pcl::PointCloud<PointT> &static_pc, 
                                          pcl::PointCloud<PointT> &dynamic_pc) {
    if (first_flag) {
        first_flag = false;
        UpdateFilter(input_pc, curr_rot, curr_pos);
        return;
    }

    Reset(input_pc);

    curr_range_img = cv::Mat(img_rows, img_cols, CV_32FC1, cv::Scalar::all(255));
    curr_count_img = cv::Mat(img_rows, img_cols, CV_32FC1, cv::Scalar::all(0));
    curr_index_img = cv::Mat(img_rows, img_cols, CV_32FC1, cv::Scalar::all(0));
    RangeImageProjection();

    int hist_num = hist_pc.size();
    for (int hist_idx = 0; hist_idx < hist_num; hist_idx++) {
        pcl::PointCloud<PointT> temp_hist_pc_world;

        M4D trans_curr = M4D::Identity();
        trans_curr.block<3, 3>(0, 0) = curr_rot;
        trans_curr(0, 3) = curr_pos[0];
        trans_curr(1, 3) = curr_pos[1];
        trans_curr(2, 3) = curr_pos[2];

        M4D trans_hist = M4D::Identity();
        trans_hist.block<3, 3>(0, 0) = hist_rot[hist_idx];
        trans_hist(0, 3) = hist_pos[hist_idx][0];
        trans_hist(1, 3) = hist_pos[hist_idx][1];
        trans_hist(2, 3) = hist_pos[hist_idx][2];

        M4D trans_final = trans_curr.inverse() * trans_hist;

        pcl::transformPointCloud(hist_pc[hist_idx], temp_hist_pc_world, trans_final);

        hist_range_img = cv::Mat(img_rows, img_cols, CV_32FC1, cv::Scalar::all(255));
        RangeImageProjection(temp_hist_pc_world);
    
        diff_range_img = cv::Mat(img_rows, img_cols, CV_32FC1, cv::Scalar::all(0));
        cv::absdiff(curr_range_img, hist_range_img, diff_range_img);

        for (int row = 0; row < img_rows; row++) {
            for (int col = 0; col < img_cols; col++) {
                float this_diff = diff_range_img.at<float>(row, col);
                float this_range = curr_range_img.at<float>(row, col);
                if((this_diff > this_range * adaptive_coeff || 
                    this_diff > adaptive_diff) && 
                    this_diff < 100) {
                    curr_count_img.at<int>(row, col)++;
                }
            }
        }
    }

    if (pub_range_img) {
        std_msgs::Header header;
        header.frame_id='world';
        header.stamp = ros::Time().fromSec(input_pc.header.stamp);
        
        cv::Mat normalized_range, u8_range, color_map;
        sensor_msgs::ImagePtr curr_img_msg, hist_img_msg, diff_img_msg;
        
        cv::normalize(curr_range_img, normalized_range, 255, 0, cv::NORM_MINMAX);
        normalized_range.convertTo(u8_range, CV_8UC1);
        cv::applyColorMap(u8_range, color_map, cv::COLORMAP_JET);
        curr_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color_map).toImageMsg();
        pub_curr_range_img.publish(curr_img_msg);
        
        cv::normalize(hist_range_img, normalized_range, 255, 0, cv::NORM_MINMAX);
        normalized_range.convertTo(u8_range, CV_8UC1);
        cv::applyColorMap(u8_range, color_map, cv::COLORMAP_JET);
        hist_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color_map).toImageMsg();
        pub_hist_range_img.publish(hist_img_msg);
        
        cv::normalize(diff_range_img, normalized_range, 255, 0, cv::NORM_MINMAX);
        normalized_range.convertTo(u8_range, CV_8UC1);
        cv::applyColorMap(u8_range, color_map, cv::COLORMAP_JET);
        diff_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color_map).toImageMsg();
        pub_diff_range_img.publish(diff_img_msg);
    }

    dyn_threshold = hist_pc.size() * dyn_ratio;
    for (int row = 0; row < img_rows; row++) {
        for (int col = 0; col < img_cols; col++) {
            if(curr_count_img.at<int>(row, col) >= dyn_threshold && input_pc.points[curr_index_img.at<int>(row, col)].z < 3){
                idx_lable[curr_index_img.at<int>(row, col)] = 1;
            }
        }
    }
    
    if (use_check || use_cluster) {
        CreateVoxelMap();

        if (use_check) {
            FiltCandidateDynPoints();
        }

        if (use_cluster) {
            ExtractCluster();
        }
    }

    dynamic_pc.clear();
    static_pc.clear();
    for (int i = 0; i < curr_pc_num; i++) {
        if (idx_lable[i]) {
            dynamic_pc.points.emplace_back(curr_pc.points[i]);
        } else {
            static_pc.points.emplace_back(curr_pc.points[i]);
        }
    }

    // UpdateFilter(static_pc, curr_rot, curr_pos);
    // UpdateFilter(input_pc, curr_rot, curr_pos);
}

template <typename PointT>
void DynamicObjectFilter<PointT>::UpdateFilter(const pcl::PointCloud<PointT> &curr_static_points, 
                                               const M3D curr_rot, 
                                               const V3D curr_pos) {
    hist_pc.emplace_back(curr_static_points);
    if (hist_pc.size() > window_size) {
        hist_pc.pop_front();
    }
    hist_rot.emplace_back(curr_rot);
    if (hist_rot.size() > window_size) {
        hist_rot.pop_front();
    }
    hist_pos.emplace_back(curr_pos);
    if (hist_pos.size() > window_size) {
        hist_pos.pop_front();
    }
}

} // namespace dof

#endif
#ifndef _DOF_UTILS_
#define _DOF_UTILS_

#include <iostream>
#include <functional>
#include <vector>
#include <deque>
#include <execution>

#include "utils.h"
#include "common_lib.h"
#include "dyn_obj_filter/tsl/robin_map.h"

namespace dof {

# define M_PI 3.14159265358979323846

inline double CalculateDistance2(PointType p1, PointType p2) {
    return (p1.x - p2.x) * (p1.x - p2.x) 
         + (p1.y - p2.y) * (p1.y - p2.y) 
         + (p1.z - p2.z) * (p1.z - p2.z); 
}

struct Voxel {
public:
    int x, y, z;
    
    Voxel(int vx, int vy, int vz, float revolusion)
     : x(vx / revolusion), y(vy / revolusion), z(vz / revolusion) {}
     
    Voxel(int vx, int vy, int vz, float revolusion, int x_near_block, int y_near_block, int z_near_block)
     : x(vx / revolusion + x_near_block), y(vy / revolusion + y_near_block), z(vz / revolusion + z_near_block) {}
    
    bool operator==(const Voxel &other) const {
        return this->x == other.x 
            && this->y == other.y 
            && this->z == other.z;
    }
};

struct VoxelBlock {
public:
    VoxelBlock() = default;
    ~VoxelBlock() = default;

    void AddPoint(const int idx, const PointType pt, const bool is_dyn) {
        points_num++;
        idx_pt.emplace_back(idx);
        pts.emplace_back(pt);

        if (is_dyn) {
            dyn_points_num++;
            idx_dyn_pt.emplace_back(idx);
            dyn_pts.emplace_back(pt);
        }
    }

    bool SearchDynNeighbor(const PointType pt, const int idx, const double search_radius) {
        for (int i = 0; i < dyn_points_num; i++) {
            double distance = CalculateDistance2(pt, dyn_pts[i]);
            if (distance < search_radius * search_radius && idx != idx_dyn_pt[i]) {
                idx_lable[idx] = 2;
                idx_lable[i] = 2;
                return true;
            }
        }
        return false;
    }

    void Reset() {
        int points_num = 0;
        idx_pt.clear();
        pts.clear();

        dyn_points_num = 0;
        idx_dyn_pt.clear();
        dyn_pts.clear();

        processed_flag = 0;
    }

public:
    int points_num = 0;
    std::vector<int> idx_pt;
    std::vector<PointType> pts;

    int dyn_points_num = 0;
    std::vector<int> idx_dyn_pt;
    std::vector<PointType> dyn_pts;

    bool processed_flag = 0;
};

struct VoxelHash {
    std::size_t operator()(const Voxel &vox) const {
        const size_t kP_x = 73856093;
        const size_t kP_y = 19349669;
        const size_t kP_z = 83492791;
        return vox.x * kP_x + vox.y * kP_y + vox.z * kP_z;
    }
};

typedef tsl::robin_map<Voxel, VoxelBlock, VoxelHash> VoxelHashMap;

}   // namespace dof

#endif
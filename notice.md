代码实现主要在/include部分，在/src/li_odometry.cc的```void LIOdometry::Run()```中调用。  
## 1. 地面分割
实现地面分割，在/include/ground_seperator.hpp   
/include/li_odometry.h中定义  
```
std::shared_ptr<GroundSeperator<PointType>> ground_seperator = nullptr;
```
/src/li_odometry.cc中做分割
```
ground_seperator->EstimateGround(*scan_undistort, *ground_body, *scan_down_body);
```
地面约束
```
void LIOdometry::GroundObsModel(state_ikfom &s, esekfom::dyn_share_datastruct<double> &esekfom_data)
```
将地面约束融合
```
void LIOdometry::ObsModel(state_ikfom &s, esekfom::dyn_share_datastruct<double> &esekfom_data)
```

## 2. 哈希表
实现哈希表，在/include/tsl     
/include/dyn_obj_filter.hpp中  
```
typedef tsl::robin_map<Voxel, VoxelBlock, VoxelHash> VoxelHashMap;
```
**Voxel**:键  
**VoxelBlock**:值  
**VoxelHash**:哈希映射函数

## 3. 动态物体滤除  
实现动态物体滤除，在/include/dyn_obj_filter.hpp
### 3.1. 可见性检测
将当前点云转成距离图像  
```
void DynamicObjectFilter::RangeImageProjection()
```
将滑动窗口的历史点云转成距离图像  
```
for (sliding window loop) {
  void DynamicObjectFilter::RangeImageProjection(const CloudType &input_pc)
}
```
### 3.2. 建立哈希地图
将当前点云建立哈希地图，用于做快速筛选过滤和聚类
```
void DynamicObjectFilter::CreateVoxelMap()
```
### 3.3. 筛选过滤
半径搜索，有没有邻近的动态点
```
void DynamicObjectFilter::FiltCandidateDynPoint()
```
### 3.4. 聚类
```
void DynamicObjectFilter::ExtractCluster()
void DynamicObjectFilter::ExtendNearBlock(int idx)
```
### 3.5. 重要参数
.yaml文件中
```
dof:
  flag: 
    pub_range_img: false
    use_check: true
    use_cluster: true
  detection:
    FOV_V_UP: 15
    FOV_V_DOWN: -15
    FOV_H: 360
    image_res: 1.0
    window_size: 5
    adaptive_coeff: 0.05
    adaptive_diff: 0.03
    dyn_ratio: 0.6
  check:
    search_radius: 0.1
  cluster:
    revolusion: 1.0
    extend_block: 1
    extend_range: 0.3
    dyn_num_threshold: 5
    dyn_ratio_threshold: 0.05
```

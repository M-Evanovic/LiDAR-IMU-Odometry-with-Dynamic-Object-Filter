# LiDAR-IMU Odometry with Dynamic Object Filter  
## Environment
测试环境：Ubuntu 20.04 and [ROS noetic](https://wiki.ros.org/noetic/Installation/Ubuntu)  

## Method
1. 框架主体：  
框架就是主流的[Fast-lio](https://github.com/hku-mars/FAST_LIO) ，不多介绍。  
2. 动态物体滤除思路：  
- 通过哈希聚类快速提取物体。与其它聚类方法对比[My Clustering](https://github.com/M-Evanovic/Point-Cloud-Clustering)。  
- 将当前扫描投影成距离图像，再根据当前帧的先验位姿将滑动窗口的历史扫描转换到当前坐标并投影成距离图像。对比各距离图像可得到动态点。这部分可参考[My Removert](https://github.com/M-Evanovic/Remove-then-Revert)。  
- 检索各个聚类中动态点数量及占比判断聚类是否为动态聚类。
3. 融合思路：
在获得当前先验位姿后进行动态物体滤除处理，再进行点云配准，然后滤波更新状态和滤除器滑窗更新。

## Function
重要模块和函数的介绍都放在[notice.md](https://github.com/M-Evanovic/lio-ldof/blob/main/notice.md)。

## Compile
把package下载后  
```
mv package /workspace/src
cd /workspace
catkin_make
source ./devel/setup.bash
```

## Start
Launch the node:  
```
roslaunch lio_ldof mapping_yourdevice.launch
```
and then play your rosbag:
```
rosbag play yourbag.bag
```

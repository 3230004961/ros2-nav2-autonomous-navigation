# ROS 2 Autonomous Navigation

基于 ROS 2 Jazzy、Gazebo Harmonic 和 Nav2 的 TurtleBot3 自主导航仿真项目。

## 项目功能

- 自定义 Gazebo 室内仿真环境
- LiDAR 激光雷达与 ROS-Gazebo 通信
- SLAM Toolbox 二维栅格地图构建
- Nav2 全局路径规划和局部运动控制
- 独立的 C++ 激光雷达反应式避障算法

## 技术栈

- Ubuntu 24.04
- ROS 2 Jazzy
- C++17 / rclcpp / rclcpp_action
- Gazebo Harmonic
- Nav2 / SLAM Toolbox
- RViz2

## 项目结构

- src/：C++ 导航任务与避障节点
- launch/：仿真启动文件
- worlds/：Gazebo 室内场景
- maps/：SLAM 栅格地图
- config/：Nav2 参数文件

## 核心实现

### 1. 激光避障

通过 LaserScan 获取障碍物距离，基于左右扇区空间距离进行转向决策，利用状态机实现转向、绕障和目标跟踪。

### 2. 自主导航

使用 SLAM Toolbox 建图，AMCL 定位，Nav2 进行全局规划、局部控制和静态障碍物绕行。

### 3. C++ 多目标导航

基于 ROS 2 Action 客户端调用 NavigateToPose，实现连续目标下发、反馈监听、导航结果处理和任务取消。

## 测试结果



## 演示视频

https://www.bilibili.com/video/BV1Lf1EzyETr/?vd_source=4a673deae9daff1aba73d4c16377055e

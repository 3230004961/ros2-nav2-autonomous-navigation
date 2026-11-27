# ROS 2 移动机器人自主探索与导航系统

基于 ROS 2 Jazzy、Gazebo Harmonic、SLAM Toolbox 和 Nav2 的移动机器人自主探索项目。

使用 TurtleBot3 Burger 仿真机器人，在未知环境中实现自主建图、Frontier 探索、导航目标选择、环境边界检测及探索完成判断。

## 一、项目演示

- 仿真平台：Gazebo Harmonic
- 可视化工具：RViz
- 机器人模型：TurtleBot3 Burger
- 演示视频：待补充本次录制的视频链接

## 二、核心功能

### 1. SLAM 自主建图

- 使用 SLAM Toolbox 实时构建二维 OccupancyGrid 地图。
- 通过激光雷达获取环境信息。
- 实时更新地图和机器人位姿。

### 2. Frontier 自主探索

基于 Frontier-based Exploration 实现自主探索算法。

- 从栅格地图中提取已知自由区域与未知区域的边界。
- 使用 BFS 搜索机器人可达区域。
- 通过八邻域聚类提取 Frontier 候选目标。
- 根据信息增益和路径距离进行目标评分。
- 考虑目标方向和转向代价。
- 过滤已访问及导航失败的目标。
- 通过 Nav2 NavigateToPose Action 执行导航。

### 3. 自主环境边界检测

独立开发 ROS 2 Python 边界检测节点。

- 基于 OccupancyGrid 识别环境外围封闭区域。
- 提取有效自由空间。
- 计算封闭区域面积和地图覆盖率。
- 通过连续地图更新判断边界稳定性。
- 发布结构化边界状态。
- 使用 MarkerArray 在 RViz 中显示检测结果。

主要通信话题：

- `/exploration_boundary/status`
- `/exploration_boundary/markers`
- `/exploration_boundary/classification_markers`

### 4. 自主探索完成判断

将边界检测结果接入 Frontier 探索器，实现自主完成判断。

默认完成条件：

| 条件 | 参数 |
|---|---:|
| 地图覆盖率 | 95% |
| 边界稳定时间 | 5 秒 |
| 完成证据持续时间 | 10 秒 |
| 有效完成观测次数 | 3 次 |
| 机器人停止验证 | 通过里程计确认 |

达到完成条件后，取消导航目标，验证机器人停止，并输出任务完成状态。

### 5. 仿真与可视化

- Gazebo Harmonic 仿真环境。
- RViz 实时显示地图和激光雷达数据。
- 显示机器人探索过程及导航目标。
- 显示环境边界和分类结果。
- 支持规则地图与复杂地图场景。

## 三、技术栈

| 技术 | 版本或用途 |
|---|---|
| Ubuntu | 24.04 |
| ROS 2 | Jazzy |
| C++ | C++17 |
| Python | Python 3 |
| Gazebo | Harmonic |
| SLAM | SLAM Toolbox |
| Navigation | Nav2 |
| Visualization | RViz |
| Robot | TurtleBot3 Burger |
| Build | CMake / colcon |

## 四、项目目录

```text
laser_avoidance/
├── config/             # SLAM 与 Nav2 参数
├── launch/             # ROS 2 启动文件
├── maps/               # 地图资源
├── scripts/            # 自主探索与边界检测节点
├── src/                # C++ 功能节点
├── tools/              # 一键启动脚本与 RViz 配置
├── worlds/             # Gazebo 仿真场景
├── CMakeLists.txt
├── package.xml
├── LICENSE
└── README.md
```

## 五、环境安装与编译

运行环境：

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic
- Nav2
- SLAM Toolbox
- TurtleBot3
- ros_gz

创建工作空间：

```bash
mkdir -p ~/robot_avoidance_ws/src
```

下载项目：

```bash
cd ~/robot_avoidance_ws/src
git clone <GitHub仓库地址> laser_avoidance
```

安装依赖：

```bash
cd ~/robot_avoidance_ws

source /opt/ros/jazzy/setup.bash

rosdep install --from-paths src --ignore-src -r -y
```

编译：

```bash
colcon build --packages-select laser_avoidance

source install/setup.bash
```

## 六、运行项目

启动规则基准地图自主探索：

```bash
bash ~/robot_avoidance_ws/src/laser_avoidance/tools/run_exploration.sh baseline
```

系统自动启动 Gazebo、SLAM Toolbox、Nav2、边界检测器、Frontier 探索器和 RViz。

机器人从未知环境开始自主探索，持续更新地图、选择导航目标，并根据边界状态和地图覆盖率判断探索是否完成。

## 七、仿真结果

规则基准地图的一次成功运行记录：

| 指标 | 结果 |
|---|---:|
| 导航目标成功次数 | 33 |
| 导航目标尝试次数 | 34 |
| 累计派发路径距离 | 62.45 m |
| 重试目标派发次数 | 0 |
| 总运行时间 | 559.51 秒 |
| 任务退出码 | 0 |

演示视频：https://www.bilibili.com/video/BV1Lf1EzyETr/?spm_id_from=333.1387.homepage.video_card.click&vd_source=4a673deae9daff1aba73d4c16377055e
该数据为一次成功仿真运行记录，不代表所有环境下的性能保证。

## 八、项目技术特点

1. 自主实现 Frontier 搜索、BFS 可达区域搜索和目标评分逻辑。
2. 实现 ROS 2 边界检测节点及结构化状态发布。
3. 集成 SLAM Toolbox 与 Nav2，实现完整自主探索流程。
4. 实现覆盖率判断、导航目标取消及机器人停止验证。
5. 使用 Gazebo 和 RViz 完成自主探索仿真验证。

## License

Apache License 2.0

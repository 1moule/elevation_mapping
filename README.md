# Robot-Centric Elevation Mapping

## PORT TO ROS2 Humble

This is a quick port of [Robot-Centric Elevation Mapping](https://github.com/ANYbotics/elevation_mapping) to ROS2 based on Aber-CRANC's [tf2 branch](https://github.com/Aber-CRANC/elevation_mapping/tree/tf2).

Tested in ROS2 humble.

Port TODO list:
- [x] Port of barebone functionality
- [x] Fix all new bugs created during porting
- [x] Timers
- [x] Services
- [ ] Fix multithreading
- [ ] Demos
- [x] Testing

Known Issues:
- The current ROS2 interpretation of filter chain causes issues with parameter redeclaration when using more than 1 postprocessing thread.
- robot_pose_with_covariance_topic: directly take a [nav_msgs/Odometry] msg topic not a [geometry_msgs/PoseWithCovarianceStamped.msg]

Dependencies:
- [grid_map](https://github.com/ANYbotics/grid_map/tree/humble) - humble branch 
- [kindr](https://github.com/ANYbotics/kindr) native C build using Cmake.
- [kindr_ros](https://github.com/SivertHavso/kindr_ros/tree/galactic) - ros2 galactic branch

## Overview

This is a [ROS2] package developed for elevation mapping with a mobile robot. The software is designed for (local) navigation tasks with robots which are equipped with a pose estimation (e.g. IMU & odometry) and a distance sensor (e.g. structured light (Kinect, RealSense), laser range sensor, stereo camera). The provided elevation map is limited around the robot and reflects the pose uncertainty that is aggregated through the motion of the robot (robot-centric mapping). This method is developed to explicitly handle drift of the robot pose estimation.

This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

The source code is released under a [BSD 3-Clause license](LICENSE).

**Author: Péter Fankhauser<br />
Co-Author: Maximilian Wulf<br />
Affiliation: [ANYbotics](https://www.anybotics.com/)<br />
Maintainer: Maximilian Wulf, mwulf@anybotics.com, Magnus Gärtner, mgaertner@anybotics.com<br />**

This projected was initially developed at ETH Zurich (Autonomous Systems Lab & Robotic Systems Lab).

[This work is conducted as part of ANYmal Research, a community to advance legged robotics.](https://www.anymal-research.org/)

<img alt="Elevation Map Example" src="elevation_mapping_demos/doc/elevation_map.jpg" width="700">


Videos of the elevation mapping software in use:

<a alt="StarlETH Kinect elevation mapping" href="https://www.youtube.com/watch?v=I9eP8GrMyNQ"><img src="elevation_mapping_demos/doc/starleth_kinect.jpg" align="left" width="180" ></a>
<a alt="ANYmal outdoor terrain mapping" href="https://www.youtube.com/watch?v=iVMsQPTM65M"><img src="elevation_mapping_demos/doc/anymal_forrest.jpg" align="left" width="180" ></a>
<a alt="ANYmal rough-terrain locomotion planner" href="https://www.youtube.com/watch?v=CpzQu25iLa0"><img src="elevation_mapping_demos/doc/anymal_locomotion_planner.jpg" align="left" width="180" ></a>
<a alt="ANYmal outdoor stair climbing" href="https://www.youtube.com/watch?v=vSveQrJLRTo"><img src="elevation_mapping_demos/doc/anymal_outdoor_stairs.jpg" width="180" ></a>

Ported to ROS2 at Teknolus : Under the support of Scientific and Technological Research Council of Türkiye - TEYDEB 1501 Project Number : 3220580

## MID360 / FAST-LIO2 定制修改

本仓库当前分支包含针对 Livox MID360 和 FAST-LIO2 的高程图接入与边缘稳定性修改。以下内容描述已经实现并通过实机与固定 Bag 验证的行为。高程图修改不改变 Livox 驱动或 FAST-LIO2 的时间戳、点云发布频率和里程计配置。

### 当前接入关系

| 用途 | Topic | Frame | 实机验证频率 |
| --- | --- | --- | --- |
| Livox 点云 | `/livox/lidar` | Livox 驱动配置 | 约 10 Hz |
| Livox 内置 IMU | `/livox/imu` | Livox 驱动配置 | 约 200 Hz |
| FAST-LIO2 去畸变点云 | `/fastlio2/body_cloud` | `body` | 约 10 Hz |
| FAST-LIO2 里程计 | `/fastlio2/lio_odom` | `world` -> `body` | 约 10 Hz |
| 融合高程图 | `/elevation_map` | `world` | 约 20 Hz |

`elevation_mapping` 使用 `/fastlio2/body_cloud` 和 `/fastlio2/lio_odom`，地图坐标系为 `world`，机器人坐标系为 `body`。滚动地图大小为 3 m x 3 m，输出栅格分辨率为 0.04 m。表中频率是当前硬件接入时的观测值，不是算法强制保证值。

### 已实现的建图修改

- 边缘感知融合：避免跨越明显高度差的观测在融合时被平均成斜面。
- 边缘安全的发布孔洞填补：只使用高度一致且支持数足够的邻域填补小孔洞。
- 自适应低表面恢复：下台阶或下高台时，在空间与时间支持满足条件后更快接受新的低表面。
- 扫描内高度模式提取：先按当前扫描的点数与 XY 空间分布筛选高度模式，抑制孤立障碍点。
- 每个栅格最多维护两个内部持久高度模式；主模式切换必须连续 3 帧比当前模式多至少 1 个 XY bin。
- 0.5 s 内没有更新的内部高度模式会失效，防止旧模式长期控制输出。
- 地图移动、清空、几何变化和原始地图替换时，多模式状态会同步移动、重置或重新对齐。
- 可选定时器关闭时，节点仍可正常退出；固定 ROI 回放分析可重复检查边缘跳变、覆盖率、平地稳定性、边缘宽度和 CPU 开销。

这些功能由 `enable_multimodal_cells` 控制。设置为 `false` 时使用原有的单高度处理路径，便于回归对比。

### MID360 已验证参数

参数文件为 [`config/robots/fastlio_mid360.yaml`](config/robots/fastlio_mid360.yaml)。

| 参数 | 当前值 | 作用 |
| --- | ---: | --- |
| `edge_aware_fusion` | `true` | 开启边缘感知融合 |
| `fusion_height_difference_threshold` | `0.05` | 融合高度差阈值，单位 m |
| `enable_multimodal_cells` | `true` | 开启内部双高度模式 |
| `multimodal_height_separation` | `0.05` | 两个高度模式的最小间距，单位 m |
| `multimodal_min_points` | `3` | 单个扫描内模式的最少点数 |
| `multimodal_min_xy_bins` | `2` | 单个扫描内模式的最少 XY 支持 bin 数 |
| `multimodal_switch_margin_bins` | `1` | 主模式切换所需的支持优势 |
| `multimodal_switch_confirmations` | `3` | 主模式切换所需的连续确认帧数 |
| `multimodal_stale_timeout` | `0.5` | 模式失效时间，单位 s |
| `enable_adaptive_lower_surface` | `true` | 开启低表面恢复 |
| `lower_surface_neighbor_radius` | `0.12` | 低表面邻域半径，单位 m |
| `lower_surface_min_support` | `4` | 低表面邻域最少支持数 |
| `lower_surface_min_candidate_support` | `2` | 候选低表面最少支持数 |
| `lower_surface_recovery_count` | `2` | 低表面恢复确认次数 |
| `lower_surface_height_threshold` | `0.05` | 低表面高度差阈值，单位 m |
| `lower_surface_max_time_gap` | `0.25` | 低表面观测最大时间间隔，单位 s |
| `enable_fused_map_hole_filling` | `true` | 开启融合图小孔洞填补 |
| `fused_map_hole_filling_radius` | `0.12` | 孔洞填补邻域半径，单位 m |
| `fused_map_hole_filling_min_support` | `4` | 孔洞填补最少支持数 |
| `fused_map_hole_filling_height_threshold` | `0.05` | 邻域高度一致性阈值，单位 m |
| `enable_visibility_cleanup` | `false` | 不执行射线可见性清理 |
| `enable_continuous_cleanup` | `false` | 不执行连续穿透点清理 |

### 编译与运行

在工作空间根目录执行：

```bash
cd /home/guanlin/catkin_ws
./build_elevation_mapping.sh
./run_fastlio_elevation_mapping.sh --with-elevation-rviz
```

`build_elevation_mapping.sh` 会检查所需二进制包，已安装的包会跳过；已经安装到 `/usr/local` 的 `kindr` 也不会重复编译。运行仓库当前的 `MID360_config.json` 时，连接雷达的主机网卡需要拥有 `192.168.1.1/24`，雷达地址为 `192.168.1.123`。

开发者只编译并测试本包时可执行：

```bash
cd /home/guanlin/catkin_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --base-paths src/elevation_mapping \
  --packages-select elevation_mapping \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
./build/elevation_mapping/test_elevation_mapping_edge_aware_map --gtest_color=no
./build/elevation_mapping/test_elevation_mapping_lifecycle --gtest_color=no
python3 src/elevation_mapping/scripts/analyze_elevation_bag.py --self-test
bash src/elevation_mapping/test/run_elevation_bag_regression_test.sh
```

### 固定 Bag 回放验证

固定输入为 `/home/guanlin/catkin_ws/rosbag2_2026_07_25-09_22_56`。回归流程使用一个关闭多模式功能的 baseline 和三个开启功能的 candidate，并为每次回放分配独立的 `ROS_DOMAIN_ID`；脚本同时设置 `ROS_LOCALHOST_ONLY=1`，不会接入实机所在的 ROS domain。输出目录必须不存在。

```bash
cd /home/guanlin/catkin_ws/src/elevation_mapping
scripts/run_elevation_bag_regression.sh \
  /home/guanlin/catkin_ws/rosbag2_2026_07_25-09_22_56 \
  /tmp/elevation_baseline_092256 false 77
scripts/run_elevation_bag_regression.sh \
  /home/guanlin/catkin_ws/rosbag2_2026_07_25-09_22_56 \
  /tmp/elevation_candidate_092256_1 true 78
scripts/run_elevation_bag_regression.sh \
  /home/guanlin/catkin_ws/rosbag2_2026_07_25-09_22_56 \
  /tmp/elevation_candidate_092256_2 true 79
scripts/run_elevation_bag_regression.sh \
  /home/guanlin/catkin_ws/rosbag2_2026_07_25-09_22_56 \
  /tmp/elevation_candidate_092256_3 true 80

python3 scripts/analyze_elevation_bag.py \
  --compare /tmp/elevation_baseline_092256/metrics.json \
  /tmp/elevation_candidate_092256_1/metrics.json \
  /tmp/elevation_candidate_092256_2/metrics.json \
  /tmp/elevation_candidate_092256_3/metrics.json
```

最终验证结果：

| 指标 | 结果 |
| --- | ---: |
| `vertical_switch_reduction` | `0.9864864864864865` |
| `finite_coverage_ratio` | `1.0` |
| `flat_variation_ratio` | `0.9987039357152114` |
| `edge_transition_width_cells` | `0.0` |
| `candidate_runs_identical` | `true` |
| `cpu_increase_ratio` | `0.006007751937984418` |

主要 ROI 覆盖 Bag 中占主导的平地和地面凸起高台；周围孤立障碍物仅作为次要诊断，不是本次验收重点。

### 语义、限制与后续工作

- TF 只负责把点云对齐到 `world` 并移动滚动地图，不会直接推算雷达当前看不到区域的高程。
- 已经观测过的栅格在被遮挡后可能继续保留；从未观测到的隐藏区域不会仅凭 TF 获得高度。
- 发布阶段可以用高度一致的邻域填补少量孔洞，但不会跨越明显边缘大范围外推。
- 当前 MID360 参数关闭了 `enable_visibility_cleanup`，因此不会主动按视线删除被遮挡的旧表面。
- 对外发布的 `/elevation_map` 仍是单值 2.5D 高程图。内部双高度模式可以减少上下表面切换，但一个 0.04 m 输出栅格仍不能完整表示同一 `(x, y)` 内的垂直墙面。
- 持久化 `3x3` 子栅格归属、按坐标选择高低模式，以及对边界歧义查询返回无效值，均是后续候选方案，**当前尚未实现**。

### 设计文档

- [多模式边缘高程设计](docs/superpowers/specs/2026-07-25-multimodal-edge-elevation-design.md)
- [多模式边缘高程实施计划](docs/superpowers/plans/2026-07-25-multimodal-edge-elevation-implementation.md)

## Citing

The robot-centric elevation mapping methods used in this software are described in the following paper (available [here](https://doi.org/10.3929/ethz-b-000272110)). If you use this work in an academic context, please cite the following publication(s):

* > P. Fankhauser, M. Bloesch, and M. Hutter,
  > **"Probabilistic Terrain Mapping for Mobile Robots with Uncertain Localization"**,
  > in IEEE Robotics and Automation Letters (RA-L), vol. 3, no. 4, pp. 3019–3026, 2018. ([PDF](http://dx.doi.org/10.1109/LRA.2018.2849506))

        @article{Fankhauser2018ProbabilisticTerrainMapping,
          author = {Fankhauser, P{\'{e}}ter and Bloesch, Michael and Hutter, Marco},
          doi = {10.1109/LRA.2018.2849506},
          title = {Probabilistic Terrain Mapping for Mobile Robots with Uncertain Localization},
          journal = {IEEE Robotics and Automation Letters (RA-L)},
          volume = {3},
          number = {4},
          pages = {3019--3026},
          year = {2018}
        }

* > P. Fankhauser, M. Bloesch, C. Gehring, M. Hutter, and R. Siegwart,
  > **"Robot-Centric Elevation Mapping with Uncertainty Estimates"**,
  > in International Conference on Climbing and Walking Robots (CLAWAR), 2014. ([PDF](http://dx.doi.org/10.3929/ethz-a-010173654))

        @inproceedings{Fankhauser2014RobotCentricElevationMapping,
          author = {Fankhauser, P\'{e}ter and Bloesch, Michael and Gehring, Christian and Hutter, Marco and Siegwart, Roland},
          title = {Robot-Centric Elevation Mapping with Uncertainty Estimates},
          booktitle = {International Conference on Climbing and Walking Robots (CLAWAR)},
          year = {2014}
        }

## Installation

### Dependencies

This software is built on the Robotic Operating System ([ROS2]), which needs to be [installed](http://wiki.ros.org) first. Additionally, the Robot-Centric Elevation Mapping depends on following software:

- [Grid Map](https://github.com/ANYbotics/grid_map/tree/humble) (grid map library for mobile robots)
- [kindr](http://github.com/anybotics/kindr) (kinematics and dynamics library for robotics),
- [kindr_ros](https://github.com/SivertHavso/kindr_ros/tree/galactic) (ROS wrapper for kindr),
- [Point Cloud Library (PCL)](http://pointclouds.org/) (point cloud processing),
- [Eigen](http://eigen.tuxfamily.org) (linear algebra library).



### Building

In order to install the Robot-Centric Elevation Mapping, clone the latest version from this repository into your catkin workspace and compile the package using ROS.

    cd ws/src
    git clone https://github.com/Muhammad540/elevation_mapping.git
    cd ../
    colcon build

## Basic Usage

In order to get the Robot-Centric Elevation Mapping to run with your robot, you will need to adapt a few parameters. It is the easiest if duplicate and adapt all the parameter files inside the config directory of the `elevation_mapping` package. Sepcifically you should focus on the following parameter files: 

- config/robots/ground_truth_demo.yaml
- config/elevation_maps/long_range.yaml
- config/sensor+processors.[choose your sensor file]

### TurtleBot3 Waffle Simulation

You can test with TurtleBot3, by obtaining the pointcloud2 and pose data from the simulation and configuring the config/robots/ground_truth_demo.yaml file.

## Nodes

### Node: elevation_mapping

This is the main Robot-Centric Elevation Mapping node. It uses the distance sensor measurements and the pose and covariance of the robot to generate an elevation map with variance estimates.


#### Subscribed Topics

* **`/points`** ([sensor_msgs/PointCloud2])

    The distance measurements.

* **`/pose`** ([geometry_msgs/PoseWithCovarianceStamped])

    The robot pose and covariance.

* **`/tf`** ([tf2_msgs/TFMessage])

    The transformation tree.


#### Published Topics

* **`elevation_map`** ([grid_map_msgs/GridMap])

    The entire (fused) elevation map. It is published periodically (see `fused_map_publishing_rate` parameter) or after the `trigger_fusion` service is called.

* **`elevation_map_raw`** ([grid_map_msgs/GridMap])

    The entire (raw) elevation map before the fusion step.


#### Services

* **`trigger_fusion`** ([std_srvs/Empty])

    Trigger the fusing process for the entire elevation map and publish it. For example, you can trigger the map fusion step from the console with

        ros2 service call /elevation_mapping/trigger_fusion

* **`get_submap`** ([grid_map_msgs/GetGridMap])

    Get a fused elevation submap for a requested position and size. For example, you can get the fused elevation submap at position (-0.5, 0.0) and size (0.5, 1.2) described in the odom frame and save it to a text file form the console with

        ros2 service call -- /elevation_mapping/get_submap odom -0.5 0.0 0.5 1.2 []

* **`get_raw_submap`** ([grid_map_msgs/GetGridMap])

    Get a raw elevation submap for a requested position and size. For example, you can get the raw elevation submap at position (-0.5, 0.0) and size (0.5, 1.2) described in the odom frame and save it to a text file form the console with

        ros2 service call -- /elevation_mapping/get_raw_submap odom -0.5 0.0 0.5 1.2 []

* **`clear_map`** ([std_srvs/Empty])

    Initiates clearing of the entire map for resetting purposes. Trigger the map clearing with

        ros2 service call /elevation_mapping/clear_map

* **`masked_replace`** ([grid_map_msgs/SetGridMap])

    Allows for setting the individual layers of the elevation map through a service call. The layer mask can be used to only set certain cells and not the entire map. Cells containing NAN in the mask are not set, all the others are set. If the layer mask is not supplied, the entire map will be set in the intersection of both maps. The provided map can be of different size and position than the map that will be altered. An example service call to set some cells marked with a mask in the elevation layer to 0.5 is

        ros2 service call /elevation_mapping/masked_replace "map:
          info:
            header:
              seq: 3
              stamp: {secs: 3, nsecs: 80000000}
              frame_id: 'odom'
            resolution: 0.1
            length_x: 0.3
            length_y: 0.3
            pose:
              position: {x: 5.0, y: 0.0, z: 0.0}
              orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
          layers: [elevation,mask]
          basic_layers: [elevation]
          data:
          - layout:
              dim:
              - {label: 'column_index', size: 3, stride: 9}
              - {label: 'row_index', size: 3, stride: 3}
              data_offset: 0
            data: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
          - layout:
              dim:
              - {label: 'column_index', size: 3, stride: 9}
              - {label: 'row_index', size: 3, stride: 3}
              data_offset: 0
            data: [0, 0, 0, .NAN, .NAN, .NAN, 0, 0, 0]
          outer_start_index: 0
          inner_start_index: 0"

* **`save_map`** ([grid_map_msgs/ProcessFile])

    Saves the current fused grid map and raw grid map to rosbag files. Field `topic_name` must be a base name, i.e. no leading slash character (/). If field `topic_name` is empty, then `elevation_map` is used per default. Example with default topic name

        ros2 service call /elevation_mapping/save_map "file_path: '/home/integration/elevation_map.bag' topic_name: ''"

* **`load_map`** ([grid_map_msgs/ProcessFile])

    Loads the fused grid map and raw grid map from rosbag files. Field `topic_name` must be a base name, i.e. no leading slash character (/). If field `topic_name` is empty, then `elevation_map` is used per default. Example with default topic name

        ros2 service call /elevation_mapping/load_map "file_path: '/home/integration/elevation_map.bag' topic_name: ''"

* **`disable_updates`** ([std_srvs/Empty])

    Stops updating the elevation map with sensor input. Trigger the update stopping with

        ros2 service call /elevation_mapping/disable_updates {}

* **`enable_updates`** ([std_srvs/Empty])

    Start updating the elevation map with sensor input. Trigger the update starting with

        ros2 service call /elevation_mapping/enable_updates {}

#### Parameters

* **`DEPRECATED point_cloud_topic`** (string, default: "/points")

    The name of the distance measurements topic. Use input_sources instead. 
    
* **`input_sources`** (list of input sources, default: none)

    Here you specify your inputs to elevation mapping, currently "pointcloud" inputs are supported. 
    
    Example configuration:
    ```yaml
    input_sources:
        front: # A name to identify the input source
          type: pointcloud # Supported types: pointcloud
          topic: /lidar_front/depth/points
          queue_size: 1
          publish_on_update: true # Wheter to publish the elevation map after a callback from this source. 
        rear:
          type: pointcloud
          topic: /lidar_rear/depth/points
          queue_size: 5
          publish_on_update: false
    ```
    No input sources can be configured with an empty array:
    ```yaml
    input_sources: []
    ```
* **`robot_pose_topic`** (string, default: "/robot_state/pose")

    The name of the robot pose and covariance topic.

* **`base_frame_id`** (string, default: "/robot")

    The id of the robot base tf frame.

* **`map_frame_id`** (string, default: "/map")

    The id of the tf frame of the elevation map.

* **`track_point_frame_id`** (string, default: "/robot")

    The elevation map is moved along with the robot following a *track point*. This is the id of the tf frame in which the track point is defined.

* **`track_point_x`**, **`track_point_y`**, **`track_point_z`** (double, default: 0.0, 0.0, 0.0)

    The elevation map is moved along with the robot following a *track point*. This is the position of the track point in the `track_point_frame_id`.

* **`robot_pose_cache_size`** (int, default: 200, min: 0)

    The size of the robot pose cache.

* **`min_update_rate`** (double, default: 2.0)

    The mininum update rate (in Hz) at which the elevation map is updated either from new measurements or the robot pose estimates.

* **`fused_map_publishing_rate`** (double, default: 1.0)

    The rate for publishing the entire (fused) elevation map.

* **`relocate_rate`** (double, default: 3.0)

    The rate (in Hz) at which the elevation map is checked for relocation following the tracking point.

* **`length_in_x`**, **`length_in_y`** (double, default: 1.5, min: 0.0)

    The size (in m) of the elevation map.

* **`position_x`**, **`position_y`** (double, default: 0.0)

    The position of the elevation map center, in the elevation map frame. This parameter sets the planar position offsets between the generated elevation map and the frame in which it is published (`map_frame_id`). It is only useful if no `track_point_frame_id` parameter is used.

* **`resolution`** (double, default: 0.01, min: 0.0)

    The resolution (cell size in m/cell) of the elevation map.

* **`min_variance`**, **`max_variance`** (double, default: 9.0e-6, 0.01)

    The minimum and maximum values for the elevation map variance data.

* **`mahalanobis_distance_threshold`** (double, default: 2.5)

    Each cell in the elevation map has an uncertainty for its height value. Depending on the Mahalonobis distance of the existing height distribution and the new measurements, the incoming data is fused with the existing estimate, overwritten, or ignored. This parameter determines the threshold on the Mahalanobis distance which determines how the incoming measurements are processed.

* **`sensor_processor/ignore_points_above`** (double, default: inf)
    A hard threshold on the height of points introduced by the depth sensor. Points with a height over this threshold will not be considered valid during the data collection step.

* **`sensor_processor/ignore_points_below`** (double, default: -inf)
    A hard threshold on the height of points introduced by the depth sensor. Points with a height below this threshold will not be considered valid during the data collection step.

* **`multi_height_noise`** (double, default: 9.0e-7)

    Noise added to measurements that are higher than the current elevation map at that particular position. This noise-adding process is only performed if a point falls over the Mahalanobis distance threshold. A higher value is useful to adapt faster to dynamic environments (e.g., moving objects), but might cause more noise in the height estimation.

* **`min_horizontal_variance`**, **`max_horizontal_variance`** (double, default: pow(resolution / 2.0, 2), 0.5)

    The minimum and maximum values for the elevation map horizontal variance data.

* **`enable_visibility_cleanup`** (bool, default: true)

    Enable/disable a separate thread that removes elements from the map which are not visible anymore, by means of ray-tracing, originating from the sensor frame.

* **`visibility_cleanup_rate`** (double, default: 1.0)

    The rate (in Hz) at which the visibility clean-up is performed.

* **`enable_continuous_cleanup`** (bool, default: false)

    Enable/disable a continuous clean-up of the elevation map. If enabled, on arrival of each new sensor data the elevation map will be cleared and filled up only with the latest data from the sensor. When continuous clean-up is enabled, visibility clean-up will automatically be disabled since it is not needed in this case.
    
* **`num_callback_threads`** (int, default: 1, min: 1)
    The number of threads to use for processing callbacks. More threads results in higher throughput, at cost of more resource usage. 

* **`postprocessor_pipeline_name`** (string, default: postprocessor_pipeline)

    The name of the pipeline to execute for postprocessing. It expects a pipeline configuration to be loaded in the private namespace of the node under this name. 
    E.g.:
    ```
      <node pkg="elevation_mapping" type="elevation_mapping" name="elevation_mapping" output="screen">
          ...
          <ros2 param command="load" file="$(find elevation_mapping_demos)/config/postprocessor_pipeline.yaml" />
      </node>
    ```
    A pipeline is a grid_map_filter chain, see grid_map_demos/filters_demo.yaml and [ros / filters](http://wiki.ros.org/filters) for more information. 

* **`postprocessor_num_threads`** (int, default: 1, min: 1)

    The number of threads to use for asynchronous postprocessing. More threads results in higher throughput, at cost of more resource usage. 

* **`scanning_duration`** (double, default: 1.0)

    The sensor's scanning duration (in s) which is used for the visibility cleanup. Set this roughly to the duration it takes between two consecutive full scans (e.g. 0.033 for a ToF camera with 30 Hz, or 3 s for a rotating laser scanner). Depending on how dense or sparse your scans are, increase or reduce the scanning duration. Smaller values lead to faster dynamic object removal and bigger values help to reduce faulty map cleanups.

* **`sensor_cutoff_min_depth`**, **`sensor_cutoff_max_depth`** (double, default: 0.2, 2.0)

    The minimum and maximum values for the length of the distance sensor measurements. Measurements outside this interval are ignored.

* **`sensor_model_normal_factor_a`**, **`sensor_model_normal_factor_b`**, **`sensor_model_normal_factor_c`**, **`sensor_model_lateral_factor`** (double)

    The data for the sensor noise model.

## Changelog

See [Changelog]

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/anybotics/elevation_mapping/issues).

[Changelog]: CHANGELOG.rst
[ROS]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[grid_map_msgs/GridMap]: https://github.com/anybotics/grid_map/blob/master/grid_map_msgs/msg/GridMap.msg
[sensor_msgs/PointCloud2]: http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html
[geometry_msgs/PoseWithCovarianceStamped]: http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html
[tf2_msgs/TFMessage]: http://docs.ros.org/en/noetic/api/tf2_msgs/html/msg/TFMessage.html
[std_srvs/Empty]: http://docs.ros.org/api/std_srvs/html/srv/Empty.html
[grid_map_msgs/GetGridMap]: https://github.com/anybotics/grid_map/blob/master/grid_map_msgs/srv/GetGridMap.srv
[grid_map_msgs/ProcessFile]: https://github.com/ANYbotics/grid_map/blob/master/grid_map_msgs/srv/ProcessFile.srv

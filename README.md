# Modified GenZ-ICP

This repository contains a ROS2-focused modified version of GenZ-ICP for rover LiDAR odometry. It builds on the original GenZ-ICP project and adds covariance output, IMU-assisted initialization and deskewing, registration quality gates, dynamic-scene filtering, twist output, and rover/Blickfeld launch presets.

The repository name is `modified-genz-icp`. The ROS package name remains `genz_icp`, so existing ROS launch and package commands still use `genz_icp`.

![GenZ-ICP demo](pictures/GenZ-ICP.gif)

## Scope

This repo contains the modified C++ core and ROS wrappers only:

- `cpp/genz_icp`: C++ GenZ-ICP library.
- `ros`: ROS1/ROS2 wrapper package named `genz_icp`.
- `ros/ros2`: ROS2 odometry, IMU deskew, and dynamic point cloud filter nodes.
- `ros/config`: dataset presets plus the added `rover.yaml`.
- `ros/launch`: odometry, deskew, dynamic filter, and BF LiDAR pipeline launch files.

The original repository's Python bindings and Python dataset tools are not included in this cleaned repo. A local debug helper script from the modified workspace was also intentionally omitted because it is not part of the package build.

## Main Improvements

A detailed comparison against the original local copy is in [MODIFICATIONS.md](MODIFICATIONS.md). The short version:

- Publishes a 6x6 pose covariance from the ICP information matrix in `nav_msgs/Odometry`.
- Adds registration quality metrics: RMSE, weighted RMSE, correspondence count, pose delta, finite-output checks, and rejection recovery.
- Adds optional IMU angular-velocity prediction for the ICP initial rotation.
- Adds a standalone ROS2 IMU rotation deskew node with point-time parsing, gyro-bias calibration, pending cloud queueing, and optional lever-arm correction.
- Adds optional twist estimation and twist covariance in `/genz/odometry`.
- Adds yaw-search initialization, soft motion priors, robust ICP losses, trimmed ICP, map update gates, and tentative/stable map gating.
- Adds a dynamic point cloud filter node that removes unsupported transient points using fixed-frame history.
- Adds rover/Blickfeld-oriented configuration and launch files.
- Hardens point-time extraction for multiple field names and integer/float timestamp datatypes.

## Requirements

Tested target:

- Ubuntu 22.04
- ROS2 Humble
- CMake 3.16 or newer
- C++20 compiler

The C++ build system fetches or locates its core third-party dependencies, including Eigen, Sophus, TBB, and robin-map. The ROS package also depends on common ROS message packages, `tf2_ros`, `rclcpp_components`, `yaml-cpp`, `ament_index_cpp`, and `rcpputils`.

ROS1 wrapper code is still present, but the new IMU, deskew, dynamic filter, covariance/twist path, and BF LiDAR pipeline work are primarily ROS2-focused.

## Build With ROS2

```sh
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src
git clone https://github.com/<your-user>/modified-genz-icp.git
cd ~/colcon_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select genz_icp --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
source install/setup.bash
```

## Build C++ Only

```sh
cmake -S ~/colcon_ws/src/modified-genz-icp/cpp/genz_icp -B ~/colcon_ws/build/genz_icp_cpp -DCMAKE_BUILD_TYPE=Release
cmake --build ~/colcon_ws/build/genz_icp_cpp -j$(nproc)
```

## Run Odometry

For a generic point cloud topic:

```sh
ros2 launch genz_icp odometry.launch.py topic:=/points config_file:=outdoor.yaml visualize:=false
```

For the rover-oriented preset with covariance, twist, IMU prediction, yaw search, and motion prior enabled:

```sh
ros2 launch genz_icp odometry.launch.py \
  topic:=/bf_lidar/point_cloud_deskewed \
  config_file:=rover.yaml \
  base_frame:=base_link \
  odom_frame:=odom
```

Useful outputs:

- `/genz/odometry`: `nav_msgs/Odometry` with pose covariance and optional twist.
- `/genz/trajectory`: bounded `nav_msgs/Path`.
- `/genz/local_map`: debug local map, only published when visualization/debug subscribers exist.
- `/genz/planar_points` and `/genz/non_planar_points`: debug correspondence groups.

## Run IMU Rotation Deskew

```sh
ros2 launch genz_icp imu_rotation_deskew.launch.py \
  cloud_topic:=/bf_lidar/point_cloud_out \
  imu_topic:=/bf_lidar/imu_out \
  output_topic:=/bf_lidar/point_cloud_deskewed \
  point_time_field:=auto \
  point_time_unit:=nanoseconds \
  cloud_stamp_location:=end \
  deskew_reference:=middle
```

The deskew node integrates `sensor_msgs/Imu.angular_velocity`. It ignores `orientation` and `linear_acceleration`. If your IMU publishes angular velocity in degrees per second, set `imu_angular_velocity_scale:=0.017453292519943295`.

## Run Dynamic Point Cloud Filtering

```sh
ros2 launch genz_icp dynamic_pointcloud_filter.launch.py \
  input_topic:=/bf_lidar/point_cloud_deskewed \
  output_topic:=/bf_lidar/point_cloud_filtered \
  fixed_frame:=odom \
  dynamic_threshold_meters:=0.35
```

The filter transforms each cloud into a fixed frame and keeps points that have enough support in recent history. During warmup it can publish unfiltered clouds.

## Run The BF LiDAR Pipeline

The combined launch starts a static `base_link -> lidar` transform, the IMU deskew node, and GenZ-ICP odometry:

```sh
ros2 launch genz_icp bf_lidar_genz_pipeline.launch.py \
  raw_cloud_topic:=/bf_lidar/point_cloud_out \
  deskew_imu_topic:=/mavros/imu/data \
  odom_imu_topic:=/mavros/imu/data \
  config_file:=rover.yaml
```

Adjust the `lidar_tf_*` arguments and `lidar_lever_arm` to match the physical sensor mounting.

## Important Parameters

Registration quality gates:

- `enable_registration_quality_gate`
- `min_registration_correspondences`
- `registration_rmse_reject_ratio`
- `absolute_registration_rmse_limit`
- `max_registration_translation_per_frame`
- `max_registration_rotation_per_frame_deg`
- `max_consecutive_registration_rejections`

IMU prediction:

- `enable_imu_motion_prediction`
- `imu_topic`
- `imu_prediction_point_time_field`
- `imu_prediction_point_time_unit`
- `imu_prediction_cloud_stamp_location`
- `imu_prediction_deskew_reference`
- `imu_angular_velocity_scale`
- `enable_imu_prediction_gyro_bias_calibration`

Robust and dynamic-scene handling:

- `enable_yaw_search_initializer`
- `enable_motion_prior`
- `enable_robust_icp_outlier_handling`
- `robust_loss_type`
- `trimmed_icp_enabled`
- `enable_map_update_quality_gate`
- `enable_tentative_map_gating`

Odometry output:

- `publish_odom_tf`
- `publish_twist`
- `twist_in_child_frame`
- `twist_smoothing_alpha`
- `twist_linear_covariance`
- `twist_angular_covariance`

## Troubleshooting

- If IMU prediction is unavailable, check that the cloud has a point-time field such as `point_time_offset`, `time`, `timestamp`, or `t`, and that `imu_prediction_point_time_unit` matches the field units.
- If deskew waits or drops scans, increase `imu_buffer_seconds`, `max_pending_cloud_wait_seconds`, or `max_imu_gap_seconds`, then verify IMU timestamps cover the cloud scan interval.
- If `/genz/odometry` jumps, enable registration logs with `terminal_status:=true`, lower `max_registration_translation_per_frame`, or enable `enable_yaw_search_initializer`.
- If frames are wrong, confirm `base_frame`, `odom_frame`, cloud `header.frame_id`, and the static LiDAR transform.

## Citation

If you use this code, cite the original GenZ-ICP paper:

```bibtex
@ARTICLE{lee2024genzicp,
  author={Lee, Daehan and Lim, Hyungtae and Han, Soohee},
  journal={IEEE Robotics and Automation Letters (RA-L)},
  title={{GenZ-ICP: Generalizable and Degeneracy-Robust LiDAR Odometry Using an Adaptive Weighting}},
  year={2025},
  volume={10},
  number={1},
  pages={152-159},
  keywords={Localization;Mapping;SLAM},
  doi={10.1109/LRA.2024.3498779}
}
```

## License

This project is distributed under the MIT License. It retains the original KISS-ICP/GenZ-ICP copyright and modification notices and adds:

```text
Further modified by Aryaman Bhatnagar, 2026
```

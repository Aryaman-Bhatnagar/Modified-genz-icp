# Modified GenZ-ICP ROS Package

This directory contains the ROS package named `genz_icp`. The repository name can be `modified-genz-icp`, but ROS commands still refer to `genz_icp`.

For the full project overview, build instructions, modification list, citation, and license notes, see the repository-level [README.md](../README.md) and [MODIFICATIONS.md](../MODIFICATIONS.md).

## Build With ROS2

```sh
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src
git clone https://github.com/<your-user>/modified-genz-icp.git
cd ~/colcon_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select genz_icp --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
source install/setup.bash
```

## Launch Files

- `odometry.launch.py`: main GenZ-ICP odometry node.
- `imu_rotation_deskew.launch.py`: standalone IMU angular-velocity deskew node.
- `dynamic_pointcloud_filter.launch.py`: fixed-frame history filter for transient points.
- `bf_lidar_genz_pipeline.launch.py`: static TF, IMU deskew, and odometry pipeline for the BF LiDAR/rover setup.

## Common Commands

```sh
ros2 launch genz_icp odometry.launch.py topic:=/points config_file:=outdoor.yaml
```

```sh
ros2 launch genz_icp odometry.launch.py topic:=/bf_lidar/point_cloud_deskewed config_file:=rover.yaml
```

```sh
ros2 launch genz_icp imu_rotation_deskew.launch.py cloud_topic:=/bf_lidar/point_cloud_out imu_topic:=/bf_lidar/imu_out output_topic:=/bf_lidar/point_cloud_deskewed point_time_unit:=nanoseconds
```

```sh
ros2 launch genz_icp dynamic_pointcloud_filter.launch.py input_topic:=/bf_lidar/point_cloud_deskewed output_topic:=/bf_lidar/point_cloud_filtered fixed_frame:=odom
```

## ROS1

The original ROS1 wrapper is retained, but the new covariance/twist, IMU prediction, deskew, dynamic filter, and BF LiDAR pipeline work is ROS2-focused.

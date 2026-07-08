# Changes From Original GenZ-ICP

This list was produced by comparing the modified workspace package at `src/genz-icp` against the local original copy at `src/original genz icp`.

## Summary

| Area | Original package | Modified package |
| --- | --- | --- |
| Repository scope | C++, ROS, Python bindings, Python dataset tools | C++ and ROS only in the cleaned repo |
| Registration API | Pose plus planar/non-planar debug points | Pose, debug points, covariance, and registration quality metrics |
| Odometry output | Pose odometry without covariance/twist support | Pose covariance, optional twist, bounded path history |
| IMU use | No ROS2 IMU prediction path | Optional angular-velocity prediction and standalone rotation deskew |
| Robustness | Base GenZ-ICP adaptive weighting | Quality gates, yaw search, motion prior, robust losses, trimmed ICP |
| Dynamic scenes | Local map updated directly | Optional map update gate, tentative/stable map gating, dynamic cloud filter |
| Launch/config | Standard dataset presets | Adds rover preset and BF LiDAR pipeline launch |

## Added Improvements

1. Pose covariance

   The C++ registration step now estimates a 6x6 covariance from the final weighted information matrix and residual variance. ROS2 odometry copies that matrix into `nav_msgs/Odometry.pose.covariance`.

2. Registration quality reporting

   Registration now returns RMSE, weighted RMSE, correspondence count, translation delta, rotation delta, and finite-output status. The pipeline uses these values to accept, reject, or recover from bad ICP solves.

3. Registration rejection and recovery

   The pipeline can reject frames with too few correspondences, non-finite results, excessive pose jumps, or RMSE spikes. Rejected frames keep the last accepted pose/covariance and can trigger a recovery mode after consecutive failures.

4. IMU-assisted initial guess

   ROS2 odometry can subscribe to an IMU topic, integrate angular velocity between accepted scan reference times, and use that rotation as the ICP initial rotation. The implementation supports point-time fields, unit conversion, max-gap checks, max-age checks, and gyro-bias calibration.

5. Standalone IMU rotation deskew node

   `imu_rotation_deskew_node` deskews a `PointCloud2` using IMU angular velocity. It supports multiple point-time field names and integer/float timestamp datatypes, scan start/middle/end reference choices, pending cloud queueing while IMU coverage arrives, manual or calibrated gyro bias, and optional LiDAR lever-arm correction.

6. Twist publication

   ROS2 odometry can compute linear and angular velocity from consecutive accepted poses, publish it in either child or odom frame, smooth it, and populate twist covariance.

7. Yaw-search initializer

   Before ICP, the pipeline can score a list of yaw offsets against the local map and choose the best initial guess by RMSE or weighted RMSE. This helps after yaw slips or poor motion prediction.

8. Motion prior inside ICP

   Registration can add a soft prior to the normal equations, with separate XY, Z, roll/pitch, and yaw sigmas. This gives ICP a configurable pull toward the predicted motion without hard-locking the solution.

9. Robust ICP outlier handling

   Registration supports max correspondence filtering, robust losses (`none`, `huber`, `cauchy`, `tukey`), and optional trimmed ICP keep ratios.

10. Map update quality gate

    The pipeline can skip local-map insertion when registration quality is poor, even if the odometry pose itself was accepted.

11. Tentative map gating

    New points can be held in a tentative map until they are observed enough times or supported by the stable map. Yaw-based normal, relaxed, and frozen modes reduce dynamic-object contamination during sharp turns.

12. Dynamic PointCloud2 filter

    `dynamic_pointcloud_filter_node` removes transient points that lack support in recent fixed-frame history. It preserves the original cloud layout for kept points and can publish either in the original frame or fixed frame.

13. Safer timestamp and cloud handling

    ROS2 utilities now look for `point_time_offset`, `t`, `time`, and `timestamp`, support integer and floating point timestamp fields, handle zero timestamp span, and warn/fallback instead of failing hard in common malformed cases.

14. Bounded state and debug publishing

    Pose history is bounded by `max_pose_history`, trajectory publishing is bounded by `max_path_length`, and debug clouds are published only when subscribers exist.

15. Rover and BF LiDAR presets

    The modified package adds `ros/config/rover.yaml`, `imu_rotation_deskew.launch.py`, `dynamic_pointcloud_filter.launch.py`, and `bf_lidar_genz_pipeline.launch.py`.

## Fixes Applied During Repo Cleanup

- Added Aryaman Bhatnagar modification attribution to all MIT license files.
- Added the missing ROS2 `rcpputils` dependency to `ros/package.xml`.
- Added Aryaman Bhatnagar as a ROS package maintainer in the cleaned repo.
- Replaced a hardcoded workspace path in `bf_lidar_genz_pipeline.launch.py` with a package-share path to `rover.yaml`.
- Restored the C++ project version to `0.3.2`.
- Restored CMake 3.31 compatibility guards for fetched Sophus and robin-map dependencies.
- Updated `CITATION.cff` to match the RA-L citation year used in the README.
- Expanded `.gitignore` for ROS workspace build outputs.

## Removed Or Intentionally Not Included

- The original Python package and dataset tools are not part of this cleaned modified repo.
- `sixseven.py`, a local precision-inspection helper for `/genz/odometry`, was omitted because it is not part of the ROS/C++ package.
- `pictures/GenZ-ICP_visualizer.gif` from the original local copy is not present in the modified package.

## Remaining Notes

- The covariance is an ICP-derived estimate from the final weighted normal equations, not a full sensor-fusion covariance model.
- IMU prediction and deskew currently use angular velocity only; IMU orientation and linear acceleration are intentionally ignored.
- ROS1 wrapper code is retained, but the newest functionality is implemented in ROS2.

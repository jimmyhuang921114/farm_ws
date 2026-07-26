# MID-360 calibration status

This profile preserves the repository's existing `T_lidar_imu` value from the
VLP-16/FDILINK profile so the current integration can be reproduced. It has not
been confirmed as a measured MID-360-to-DECXIN or MID-360-IMU calibration.

`T_lidar_imu` maps IMU-frame points into the LiDAR frame. Before collecting
mapping data, replace it with a dated, measured calibration and verify the
LiDAR/IMU time offset. No camera extrinsic or camera intrinsic calibration is
claimed by this profile.

The launcher publishes the corresponding `base_link -> livox_frame` static TF
so GLIM can extend the single `map -> odom -> base_link` tree. The current
numeric tuple is self-inverse for its particular rotation/translation; do not
assume that property when installing a measured replacement.

The Livox PointCloud2 driver publishes `x`, `y`, `z`, `intensity`, `tag`,
`line`, and `timestamp`. `autoconf_perpoint_times` remains enabled so GLIM can
identify the `timestamp` field as relative per-point time.

# farm_sensor_bringup

Integration launches for the physical VLP-16, FDILINK AHRS, and one RealSense
D435i. No launch action runs `sudo` or changes host networking/permissions.

Camera projection is deliberately disabled until the
`velodyne -> camera_link` extrinsic is calibrated. Raw visualization does not
require that transform.

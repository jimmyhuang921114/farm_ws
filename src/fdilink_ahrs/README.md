---
date: 2024-12-29 14:57:04
---

### install rviz2

```bash
sudo apt install ros-humble-rviz-imu-plugin
```

### pub

```bash
ros2 topic pub /imu sensor_msgs/msg/Imu \
  "{header: {frame_id: 'gyro_link'}, \
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}, \
    angular_velocity: {x: 0.1, y: 0.2, z: 0.3}, \
    linear_acceleration: {x: 0.0, y: 0.0, z: 9.8}}"
```

### sub

```bash
ros2 topic echo /imu --field "orientation.x"
```

### launch

load imu sensor

```bash
ros2 launch fdilink_ahrs ahrs_driver.launch.py
ros2 topic echo /imu --field orientation
```

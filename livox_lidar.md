sudo cmake --install build
sudo ldconfig



SDK_DIR=/workspace/farm_ws/src/Livox-SDK2
SDK_BUILD="${SDK_DIR}/build"

cmake -S "${SDK_DIR}" \
      -B "${SDK_BUILD}" \
      -DCMAKE_BUILD_TYPE=Release

cmake --build "${SDK_BUILD}" \
      --parallel "$(nproc)"

sudo cmake --install "${SDK_BUILD}"

echo "/usr/local/lib" |
sudo tee /etc/ld.so.conf.d/livox-sdk2.conf >/dev/null

sudo ldconfig


cd /workspace/farm_ws/src/Livox-SDK2

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel "$(nproc)"
sudo cmake --install build
sudo ldconfig

set +u

source /opt/ros/humble/setup.bash
source /workspace/farm_ws/install/setup.bash

sudo ldconfig

ros2 launch livox_ros_driver2 msg_MID360_launch.py	

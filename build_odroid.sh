source /opt/ros/humble/setup.bash
cd autonav_ws
set MAKEFLAGS="-j1 -l1"
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --merge-install --executor sequential --symlink-install
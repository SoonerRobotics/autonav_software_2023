source /opt/ros/humble/setup.bash
cd autonav_ws
set MAKEFLAGS="-j2 -l2"
colcon build --cmake-args --executor sequential
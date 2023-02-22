# autonav_software_2023

Software for the 2023 [Intelligent Ground Vehicle Competition](http://www.igvc.org/), AutoNav challenge, the **Weeb Wagon**.  
We are using [ROS2 Humble](https://docs.ros.org/en/humble/index.html) on [Ubuntu 22.04](https://releases.ubuntu.com/22.04/).

## Dependencies

To setup all dependencies, run the following two commands. It is **CRITICAL** you do **NOT** run these commmands as **sudo**
```bash
cd setup
./setup.sh
```

## Building

```bash
source /opt/ros/humble/setup.bash
colcon build
source /install/setup.bash
```

## Manual/Remote Control

Follow the steps in [building](#building) and then run the following command
```bash
ros2 launch autonav_launch managed_manual.xml
```

Or, for the steam controller
```bash
ros2 launch autonav_launch managed_manual_steam.xml
```

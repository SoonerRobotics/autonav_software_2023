# autonav_software_2023

Software for the 2023 [Intelligent Ground Vehicle Competition](http://www.igvc.org/), AutoNav challenge, the **Weeb Wagon**.  
We are using [ROS2 Humble](https://docs.ros.org/en/humble/index.html) on [Ubuntu 22.04](https://releases.ubuntu.com/22.04/).

## Contribution

See [contribution.md](/docs/contribution.md) for more details.

## Building

```bash
source /opt/ros/humble/setup.sh

colcon build
source /install/setup.sh
```

## Manual/Remote Control

Follow the steps in [building](#building) and then run the following command
```bash
ros2 launch autonav_launch manual.xml
```

## VSCode

VSCode has a fantastic [extension](https://marketplace.visualstudio.com/items?itemName=ms-iot.vscode-ros) that supports ROS development. On top of this, running code the following way can ensure you are getting the most support.  
First, follow the [building](#building) steps and then run the following command
```bash
code .
```

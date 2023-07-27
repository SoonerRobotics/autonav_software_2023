# autonav_software_2023

![Github Workflow Status](https://img.shields.io/github/actions/workflow/status/SoonerRobotics/autonav_software_2023/compile_run.yml)

Software for the 2023 [Intelligent Ground Vehicle Competition](http://www.igvc.org/), AutoNav challenge, the **Weeb Wagon**.  
We are using [ROS2 Humble](https://docs.ros.org/en/humble/index.html) on [Ubuntu 22.04](https://releases.ubuntu.com/22.04/).

## Dependencies

To setup all dependencies, run the following two commands. It is **CRITICAL** you do **NOT** run these commmands as **sudo**
```bash
cd setup
echo "machine files.dylanzeml.in login <user> password <password>" > vectorsecrets.txt
./setup.sh
```

## Building

```bash
source /opt/ros/humble/setup.bash
cd autonav_ws
colcon build
source /install/setup.bash
```

## Autonomous/Manual

Follow the steps in [building](#building) and then run the following command
```bash
ros2 launch autonav_launch competition.xml
```
or
```bash
ros2 launch autonav_launch practice.xml
```

## Simulation

Follow the steps in [building](#building) and then run the following command
```bash
ros2 launch autonav_launch simulation.xml
```

## VSCode

To edit the software with Visual Studio Code, please install the ros extension and open VSCode through the command line via `code` after running all steps under [Building](#building). To get proper intellisense for C++, create the following file: `.vscode/c_cpp_properties.json`
```json
{
    "configurations": [
        {
            "name": "Linux",
            "includePath": [
                "${workspaceFolder}/**",
                "/opt/ros/humble/include/**"
            ],
            "defines": [],
            "compilerPath": "/usr/bin/gcc",
            "cStandard": "c17",
            "cppStandard": "gnu++17",
            "intelliSenseMode": "linux-gcc-x64"
        }
    ],
    "version": 4
}
```

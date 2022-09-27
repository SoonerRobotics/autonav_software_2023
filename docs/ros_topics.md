# ROS Topic Documentation

Below is the list of all topics and basic information about them
## /joy

[Joy](http://wiki.ros.org/joy) is the joystick topic that has information regarding the currently connected controller.

**Methods**: subscribe  
**Message Type**: [Joy.msg](https://docs.ros.org/en/api/joy/html/)  
**Nodes**
 + [controller.py](/autonav_ws/src/autonav_remote/autonav_remote/controller.py)

## /autonav/motors/input

The *motors/input* topic controls the motor speeds.

**Methods**: subscribe/publish  
**Message Type**: [MotorInput.msg](autonav_ws/src/autonav_msgs/msg/MotorInput.msg)  
**Nodes**
 + [controller.py](/autonav_ws/src/autonav_remote/autonav_remote/controller.py)
 + [can.py](/autonav_ws/src/autonav_comm/autonav_comm/can.py)

## /autonav/motors/feedback

The *motors/feedback* topic contains information regarding the motors output.

**Methods**: subscribe/publish  
**Message Type**: [MotorFeedback.msg](autonav_ws/src/autonav_msgs/msg/MotorInput.msg)  
**Nodes**
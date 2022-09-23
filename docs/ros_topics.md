# ROS Topic Documentation

Below is the list of all topics and basic information about them
## [sub] joy

[Joy](http://wiki.ros.org/joy) is the joystick topic that has information regarding the currently connected controller.

**Methods**: subscribe  
**Message Type**:  
**Nodes**
 + [controller.py](autonav_ws/src/autonav_remote/src/controller.py)

## /autonav/motors/input

The motors/input topic contains information regarding the motor inputs.

**Methods**: publish  
**Message Type**: [MotorInput.msg](autonav_ws/src/autonav_msgs/msg/MotorInput.msg)  
**Nodes**
 + [controller.py](autonav_ws/src/autonav_remote/src/controller.py)

## /autonav/motors/feedback

The motors/feedback topic contains information regarding the motor encoders. You can find the message for these [here](autonav_ws/src/autonav_msgs/msg/MotorFeedback.msg)

**Methods**: subscribe/publish  
**Message Type**: [MotorFeedback.msg](autonav_ws/src/autonav_msgs/msg/MotorInput.msg)  
**Nodes**
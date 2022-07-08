# Reachy 2021 SDK Server

ROS2 package:
* subscribing to the joints and images topics from [reachy_controllers](https://github.com/pollen-robotics/reachy_controllers), 
* publishing the new joint_goals, efforts and velocities received from the remote [sdk client](https://github.com/pollen-robotics/reachy-sdk/tree/main/reachy_sdk),
* handling grpc services from [reachy_sdk_api](https://github.com/pollen-robotics/reachy-sdk-api).

**ROS2 Version: Foxy**

Dependencies: [reachy_sdk_api](https://github.com/pollen-robotics/reachy-sdk-api),
[reachy_msgs](https://github.com/pollen-robotics/reachy_msgs),
[grpcio](https://pypi.org/project/grpcio/),
[grpcio-tools](https://pypi.org/project/grpcio-tools/),
[protobuf](https://pypi.org/project/protobuf/),
[scipy](https://pypi.org/project/scipy/)

```bash
cd ~/reachy_ws/src
git clone https://github.com/pollen-robotics/reachy_sdk_server.git
cd ~/reachy_ws/
colcon build --packages-select reachy_sdk_server
```

## Published topics

* **/joint_goals** ([sensors_msgs/msg/JointState](http://docs.ros.org/en/api/sensor_msgs/html/msg/JointState.html))
[[reachy_sdk_server](https://github.com/pollen-robotics/reachy_sdk_server/blob/master/reachy_sdk_server/reachy_sdk_server.py)] - 
Receive new goal_position, effort and velocity for the given joints from the remote client SDK.

## Subscribed topics

* **/fan_states** ([sensors_msgs/msg/FanState](https://github.com/pollen-robotics/reachy_msgs/blob/master/msg/FanState.msg))
[[reachy_sdk_server](https://github.com/pollen-robotics/reachy_sdk_server/blob/master/reachy_sdk_server/reachy_sdk_server.py)] - State of each fan in Reachy (three per arm, two in the head and one for orbita joint), published by
[joint_state_controller](https://github.com/pollen-robotics/reachy_controllers/blob/master/reachy_controllers/joint_state_controller.py). *state=True* means that the fan is turned on.

* **/joint_states** ([sensors_msgs/msg/JointState](http://docs.ros.org/en/api/sensor_msgs/html/msg/JointState.html))
[[reachy_sdk_server](https://github.com/pollen-robotics/reachy_sdk_server/blob/master/reachy_sdk_server/reachy_sdk_server.py)] - Present
position, velocity, effort and pid from each joint (both arms, orbita and antennas), published by
[joint_state_controller](https://github.com/pollen-robotics/reachy_controllers/blob/master/reachy_controllers/joint_state_controller.py).

* **/joint_temperatures** ([reachy_msgs/msg/JointTemperature](https://github.com/pollen-robotics/reachy_msgs/blob/master/msg/JointTemperature.msg))
[[reachy_sdk_server](https://github.com/pollen-robotics/reachy_sdk_server/blob/master/reachy_sdk_server/reachy_sdk_server.py)] -
Temperature from each joint (both arms, orbita and antennas), published by
[joint_state_controller](https://github.com/pollen-robotics/reachy_controllers/blob/master/reachy_controllers/joint_state_controller.py).

* **/force_sensors** ([reachy_msgs/msg/ForceSensor](https://github.com/pollen-robotics/reachy_msgs/blob/master/msg/ForceSensor.msg))
[[reachy_sdk_server](https://github.com/pollen-robotics/reachy_sdk_server/blob/master/reachy_sdk_server/reachy_sdk_server.py)] - 
Force sensor value for left and right gripper, published by
[joint_state_controller](https://github.com/pollen-robotics/reachy_controllers/blob/master/reachy_controllers/joint_state_controller.py).

* **/left_image** ([sensor_msgs/msg/CompressedImage](https://docs.ros.org/en/api/sensor_msgs/html/msg/CompressedImage.html))
[[camera_server](https://github.com/pollen-robotics/reachy_sdk_server/blob/master/reachy_sdk_server/camera_server.py)] -
Compressed image from the left camera.

* **/right_image** ([sensor_msgs/msg/CompressedImage](https://docs.ros.org/en/api/sensor_msgs/html/msg/CompressedImage.html))
[[camera_server](https://github.com/pollen-robotics/reachy_sdk_server/blob/master/reachy_sdk_server/camera_server.py)] -
Compressed image from the right camera.

## GRPC services handled
* **GetAllJointsId** [[reachy_sdk_server](https://github.com/pollen-robotics/reachy_sdk_server/blob/master/reachy_sdk_server/reachy_sdk_server.py)] - 
Return the list of all Reachy's joints ids. <br> See [joint.proto](https://github.com/pollen-robotics/reachy-sdk-api/blob/main/protos/joint.proto)
for more details. 

* **StreamJointsState** [[reachy_sdk_server](https://github.com/pollen-robotics/reachy_sdk_server/blob/master/reachy_sdk_server/reachy_sdk_server.py)] - 
Stream the requested properties of the given joints. <br> See [joint.proto](https://github.com/pollen-robotics/reachy-sdk-api/blob/main/protos/joint.proto)
for more details.

* **SendJointsCommands** [[reachy_sdk_server](https://github.com/pollen-robotics/reachy_sdk_server/blob/master/reachy_sdk_server/reachy_sdk_server.py)] -
Set the requested properties of the the given joints. <br> See [joint.proto](https://github.com/pollen-robotics/reachy-sdk-api/blob/main/protos/joint.proto)
for more details.

* **StreamJointsCommands** [[reachy_sdk_server](https://github.com/pollen-robotics/reachy_sdk_server/blob/master/reachy_sdk_server/reachy_sdk_server.py)] -
Set continuously the requested motors to the requested positions. <br> See [joint.proto](https://github.com/pollen-robotics/reachy-sdk-api/blob/main/protos/joint.proto) for more details.

* **GetAllForceSensorsId** [[reachy_sdk_server](https://github.com/pollen-robotics/reachy_sdk_server/blob/master/reachy_sdk_server/reachy_sdk_server.py)] -
Return the list of all Reachy's sensors names and ids. <br> See [sensor.proto](https://github.com/pollen-robotics/reachy-sdk-api/blob/main/protos/sensor.proto)
for more details.

* **GetSensorsState** [[reachy_sdk_server](https://github.com/pollen-robotics/reachy_sdk_server/blob/master/reachy_sdk_server/reachy_sdk_server.py)] - 
Return the state of the requested sensors.  <br> See [sensor.proto](https://github.com/pollen-robotics/reachy-sdk-api/blob/main/protos/sensor.proto)
for more details.

* **StreamSensorStates** [[reachy_sdk_server](https://github.com/pollen-robotics/reachy_sdk_server/blob/master/reachy_sdk_server/reachy_sdk_server.py)] -
Return continuously the state of the requested sensors.  <br> See [sensor.proto](https://github.com/pollen-robotics/reachy-sdk-api/blob/main/protos/sensor.proto)
for more details.

* **ComputeOrbitaIK** [[reachy_sdk_server](https://github.com/pollen-robotics/reachy_sdk_server/blob/master/reachy_sdk_server/reachy_sdk_server.py)] -
Carry out the inverse kinematics computation for Reachy's orbita neck. <br> See [orbita_kinematics.proto](https://github.com/pollen-robotics/reachy-sdk-api/blob/main/protos/orbita_kinematics.proto) for more details.

* **GetQuaternionTransform** [[reachy_sdk_server](https://github.com/pollen-robotics/reachy_sdk_server/blob/master/reachy_sdk_server/reachy_sdk_server.py)] -
Get the quaternion corresponding to a given lookVector in Reachy's orbita neck coordinate system. <br> See [orbita_kinematics.proto](https://github.com/pollen-robotics/reachy-sdk-api/blob/main/protos/orbita_kinematics.proto) for more details.

* **ComputeArmFK** [[reachy_sdk_server](https://github.com/pollen-robotics/reachy_sdk_server/blob/master/reachy_sdk_server/reachy_sdk_server.py)] -
Carry out the forward kinematics computation for Reachy's arm. <br> See [arm_kinematics.proto](https://github.com/pollen-robotics/reachy-sdk-api/blob/main/protos/arm_kinematics.proto) for more details.

* **ComputeArmIK** [[reachy_sdk_server](https://github.com/pollen-robotics/reachy_sdk_server/blob/master/reachy_sdk_server/reachy_sdk_server.py)] -
Carry out the inverse kinematics computation for Reachy's arm. <br> See [arm_kinematics.proto](https://github.com/pollen-robotics/reachy-sdk-api/blob/main/protos/arm_kinematics.proto) for more details.

* **SendFullBodyCartesianCommands** [[reachy_sdk_server](https://github.com/pollen-robotics/reachy_sdk_server/blob/master/reachy_sdk_server/reachy_sdk_server.py)] - Set the joints to the requested positions given targets in cartesian coordinate system, for both arms and head of Reachy. <br> See [fullboody_cartesian_command.proto](https://github.com/pollen-robotics/reachy-sdk-api/blob/main/protos/fullbody_cartesian_command.proto) for more details.

* **StreamFullBodyCartesianCommands** [[reachy_sdk_server](https://github.com/pollen-robotics/reachy_sdk_server/blob/master/reachy_sdk_server/reachy_sdk_server.py)] - Set continuously the joints to the requested positions given targets in cartesian coordinate system, for both arms and head of Reachy. <br> See [fullbody_cartesian_command.proto](https://github.com/pollen-robotics/reachy-sdk-api/blob/main/protos/fullbody_cartesian_command.proto) for more details.

* **GetAllFansId** [[reachy_sdk_server](https://github.com/pollen-robotics/reachy_sdk_server/blob/master/reachy_sdk_server/reachy_sdk_server.py)] - Return the id of each fan in Reachy. <br> See [fan.proto](https://github.com/pollen-robotics/reachy-sdk-api/blob/main/protos/fan.proto) for more details.

* **GetFansState** [[reachy_sdk_server](https://github.com/pollen-robotics/reachy_sdk_server/blob/master/reachy_sdk_server/reachy_sdk_server.py)] - Return the state of the requested fans. <br> See [fan.proto](https://github.com/pollen-robotics/reachy-sdk-api/blob/main/protos/fan.proto) for more details.

* **SendFansCommands** [[reachy_sdk_server](https://github.com/pollen-robotics/reachy_sdk_server/blob/master/reachy_sdk_server/reachy_sdk_server.py)] - Set the requested fans to the requested states. <br> See [fan.proto](https://github.com/pollen-robotics/reachy-sdk-api/blob/main/protos/fan.proto) for more details.

* **GetImage** [[camera_server](https://github.com/pollen-robotics/reachy_sdk_server/blob/master/reachy_sdk_server/camera_server.py)] -
Return the image of the requested Reachy's camera. <br> See [camera_reachy.proto](https://github.com/pollen-robotics/reachy-sdk-api/blob/main/protos/camera_reachy.proto) for more details.

* **SendZoomCommand** [[camera_server](https://github.com/pollen-robotics/reachy_sdk_server/blob/master/reachy_sdk_server/camera_server.py)] - Change Reachy's camera zoom speed to the requested value. <br> See [zoom_command.proto](https://github.com/pollen-robotics/reachy-sdk-api/blob/main/protos/zoom_command.proto) for more details.

* **SetZoomSpeed** [[camera_server](https://github.com/pollen-robotics/reachy_sdk_server/blob/master/reachy_sdk_server/camera_server.py)] - Change Reachy's camera zoom to the requested predefined level (either 'in', 'inter, or 'out'). <br> See [zoom_command.proto](https://github.com/pollen-robotics/reachy-sdk-api/blob/main/protos/zoom_command.proto) for more details.

* **GetMobileBasePresence** [[reachy_sdk_server](https://github.com/pollen-robotics/reachy_sdk_server/blob/master/reachy_sdk_server/reachy_sdk_server.py)] - Return if a mobile base is specified in Reachy's configuration file (*~/.reachy.yaml*). If yes, return also the mobile base version. <br> See [mobile_platform_reachy.proto](https://github.com/pollen-robotics/reachy-sdk-api/blob/main/protos/mobile_platform_reachy.proto) for more details.

## Launch files
* **reachy_sdk_server.launch.py** - Launch reachy_sdk_server node.
* **camera_server.launch.py** - Launch camera_server node.
* **reachy_camera_sdk_server.launch.py** - Launch both reachy_sdk_server and camera_server nodes.

---
This package is part of the ROS2-based software release of the version 2021 of Reachy.

Visit [pollen-robotics.com](https://pollen-robotics.com) to learn more or join our [Dicord community](https://discord.com/invite/Kg3mZHTKgs) if you have any questions or want to share your ideas.

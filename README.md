# reachy_sdk_server

ROS2 package:
* subscribing to the joints topic from [reachy_controllers](https://github.com/pollen-robotics/reachy_controllers), 
* publishing the new joint_goals, efforts and velocities received from the [sdk client](https://github.com/pollen-robotics/reachy-sdk/tree/main/reachy_sdk),
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
Contain new goal_position, effort and velocity for the given joints.

## Subscribed topics

* **/joint_states** ([sensors_msgs/msg/JointState](http://docs.ros.org/en/api/sensor_msgs/html/msg/JointState.html))
[[reachy_sdk_server](https://github.com/pollen-robotics/reachy_sdk_server/blob/master/reachy_sdk_server/reachy_sdk_server.py)] - Present
position, velocity and effort from each joint (both arms, orbita and antennas), published by
[joint_state_controller](https://github.com/pollen-robotics/reachy_controllers/blob/master/reachy_controllers/joint_state_controller.py).

* **/joint_temperatures** ([reachy_msgs/msg/JointTemperature](https://github.com/pollen-robotics/reachy_msgs/blob/master/msg/JointTemperature.msg))
[[reachy_sdk_server](https://github.com/pollen-robotics/reachy_sdk_server/blob/master/reachy_sdk_server/reachy_sdk_server.py)] -
Temperature from each joint (both arms, orbita and antennas), published by
[joint_state_controller](https://github.com/pollen-robotics/reachy_controllers/blob/master/reachy_controllers/joint_state_controller.py).

* **/force_sensors** ([reachy_msgs/msg/ForceSensor](https://github.com/pollen-robotics/reachy_msgs/blob/master/msg/ForceSensor.msg))
[[reachy_sdk_server](https://github.com/pollen-robotics/reachy_sdk_server/blob/master/reachy_sdk_server/reachy_sdk_server.py)] - 
Force sensor value for left and right gripper, published by
[joint_state_controller](https://github.com/pollen-robotics/reachy_controllers/blob/master/reachy_controllers/joint_state_controller.py).


## GRPC services handled
* **GetAllJointsId** [[reachy_sdk_server](https://github.com/pollen-robotics/reachy_sdk_server/blob/master/reachy_sdk_server/reachy_sdk_server.py)] - 
Return the list of all Reachy's joints ids. See [joint.proto](https://github.com/pollen-robotics/reachy-sdk-api/blob/main/protos/joint.proto)
for more details. 

* **StreamJointsState** [[reachy_sdk_server](https://github.com/pollen-robotics/reachy_sdk_server/blob/master/reachy_sdk_server/reachy_sdk_server.py)] - 
Stream the requested properties of the given joints. See [joint.proto](https://github.com/pollen-robotics/reachy-sdk-api/blob/main/protos/joint.proto)
for more details.

* **SendJointsCommands** [[reachy_sdk_server](https://github.com/pollen-robotics/reachy_sdk_server/blob/master/reachy_sdk_server/reachy_sdk_server.py)] -
Set the requested properties of the the given joints.

* **StreamJointsCommands** [[reachy_sdk_server](https://github.com/pollen-robotics/reachy_sdk_server/blob/master/reachy_sdk_server/reachy_sdk_server.py)] -
Set continuously the requested motors to the requested positions.

* **GetAllForceSensorsId** [[reachy_sdk_server](https://github.com/pollen-robotics/reachy_sdk_server/blob/master/reachy_sdk_server/reachy_sdk_server.py)] -
Return the list of all Reachy's sensors ids.

* **GetSensorsState** [[reachy_sdk_server](https://github.com/pollen-robotics/reachy_sdk_server/blob/master/reachy_sdk_server/reachy_sdk_server.py)] -
* **StreamSensorStates** [[reachy_sdk_server](https://github.com/pollen-robotics/reachy_sdk_server/blob/master/reachy_sdk_server/reachy_sdk_server.py)] -
* **ComputeOrbitaIK** [[reachy_sdk_server](https://github.com/pollen-robotics/reachy_sdk_server/blob/master/reachy_sdk_server/reachy_sdk_server.py)] -
* **GetQuaternionTransform** [[reachy_sdk_server](https://github.com/pollen-robotics/reachy_sdk_server/blob/master/reachy_sdk_server/reachy_sdk_server.py)] -
* **ComputeArmFK** [[reachy_sdk_server](https://github.com/pollen-robotics/reachy_sdk_server/blob/master/reachy_sdk_server/reachy_sdk_server.py)] -
* **ComputeArmFK** [[reachy_sdk_server](https://github.com/pollen-robotics/reachy_sdk_server/blob/master/reachy_sdk_server/reachy_sdk_server.py)] -
* **SendFullBodyCartesianCommands** [[reachy_sdk_server](https://github.com/pollen-robotics/reachy_sdk_server/blob/master/reachy_sdk_server/reachy_sdk_server.py)] -
* **StreamFullBodyCartesianCommands** [[reachy_sdk_server](https://github.com/pollen-robotics/reachy_sdk_server/blob/master/reachy_sdk_server/reachy_sdk_server.py)] -

## Launch files
* **reachy_sdk_server.launch.py** - Launch reachy_sdk_server node.

---
This package is part of the ROS2-based software release of the version 2021 of Reachy.

Visit [pollen-robotics.com](https://pollen-robotics.com) to learn more or visit [our forum](https://forum.pollen-robotics.com) if you have any questions.


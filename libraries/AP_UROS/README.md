# AP_UROS: micro-ROS client library

Using the micro-ROS client library in ArduPilot.

## ESP32

TODO: move this section to an appendix

Notes for building the micro-ROS client library using the package
`micro_ros_espidf_component`.

### Config

- Update `app-colcon.meta` to allow more publishers and subscribers.
  - 1 node
  - 16 pub/sub/service/client
- Update `libmicroros.mk` to add message and service interface packages.
  - Added `ardupilot` for `ardupilot_msgs`
  - Added `geographic_info` for `geographic_msgs`
  - Added `geometry2` for `tf2_msgs`
  - Added `ardupilot` for  `ardupilot_msgs`

### Build

This refers to a standalone example used to test the micro-ROS client build.

```bash
cd ./firmware/toolchain
. ./esp-idf/export.sh
```

```bash
cd ./firmware/esp32_examples/ardupilot_uros
idf.py set-target esp32
idf.py build
idf.py -p /dev/cu.usbserial-0001 flash
```

### Run

```bash
cd ./firmware/esp32_examples/ardupilot_uros
idf.py monitor
```

```bash
cd ./firmware/esp32_examples/ardupilot_uros
idf.py monitor
```

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 2019 -v6
```

## Usage

### Nodes

```bash
$ ros2 node list
/ardupilot_uros
```

```bash
$ ros2 node info /ardupilot_uros
/ardupilot_uros
  Subscribers:
    /ap/joy: sensor_msgs/msg/Joy
    /ap/tf: tf2_msgs/msg/TFMessage
  Publishers:
    /ap/battery/battery0: sensor_msgs/msg/BatteryState
    /ap/clock: rosgraph_msgs/msg/Clock
    /ap/geopose/filtered: geographic_msgs/msg/GeoPoseStamped
    /ap/navsat/navsat0: sensor_msgs/msg/NavSatFix
    /ap/pose/filtered: geometry_msgs/msg/PoseStamped
    /ap/tf_static: tf2_msgs/msg/TFMessage
    /ap/time: builtin_interfaces/msg/Time
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Service Servers:
    /ap/arm_motors: ardupilot_msgs/srv/ArmMotors
    /ardupilot_uros/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /ardupilot_uros/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /ardupilot_uros/get_parameters: rcl_interfaces/srv/GetParameters
    /ardupilot_uros/list_parameters: rcl_interfaces/srv/ListParameters
    /ardupilot_uros/set_parameters: rcl_interfaces/srv/SetParameters
  Service Clients:

  Action Servers:

  Action Clients:
```

### Topics

```bash
$ ros2 topic list
/ap/battery/battery0
/ap/clock
/ap/geopose/filtered
/ap/joy
/ap/navsat/navsat0
/ap/pose/filtered
/ap/tf
/ap/tf_static
/ap/time
/parameter_events
/rosout
```

```bash
$ ros2 topic echo /ap/geopose/filtered --once

header:
  stamp:
    sec: 550899000
    nanosec: 1143421112
  frame_id: base_link
pose:
  position:
    latitude: 0.0
    longitude: 0.0
    altitude: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0
---
```

```bash
$ ros2 topic pub /ap/tf tf2_msgs/msg/TFMessage "{transforms: [{}, {}]}" --once
publisher: beginning loop
publishing #1: tf2_msgs.msg.TFMessage(transforms=[geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id=''), child_frame_id='', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0), rotation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))), geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id=''), child_frame_id='', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0), rotation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)))])
```

```console
UROS: tf2_msgs/TFMessage with size: 2
```

### Services

```bash
$ ros2 service list
/ap/arm_motors
/ardupilot_uros/describe_parameters
/ardupilot_uros/get_parameter_types
/ardupilot_uros/get_parameters
/ardupilot_uros/list_parameters
/ardupilot_uros/set_parameters
```

```bash
$ ros2 service call /ap/arm_motors ardupilot_msgs/srv/ArmMotors "{arm: True}" 

requester: making request: ardupilot_msgs.srv.ArmMotors_Request(arm=True)

response:
ardupilot_msgs.srv.ArmMotors_Response(result=True)
```

```console
UROS: ardupilot_msgs/ArmMotors request: 1
```

### Parameters

Create

```bash
ros2 param set /ardupilot_uros GPS_TYPE 11
```

Dump

```bash
ros2 param dump /ardupilot_uros 
/ardupilot_uros:
  ros__parameters:
    GPS_TYPE: 11
```

Get

```bash
$ ros2 param get /ardupilot_uros GPS_TYPE
Integer value is: 11
```

Set

```bash
$ ros2 param get /ardupilot_uros GPS_TYPE
Integer value is: 11
```

```bash
$ ros2 param set /ardupilot_uros GPS_TYPE 1
Set parameter successful

```bash
$ ros2 param get /ardupilot_uros GPS_TYPE
Integer value is: 1
```

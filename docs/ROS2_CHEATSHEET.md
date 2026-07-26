# ROS 2 Command Cheat Sheet

## Mental Model

```text
ROS 2 node
├── publishes topics
├── subscribes to topics
├── provides or calls services
├── provides or uses actions
└── owns parameters
```

A **topic** is a continuous stream of messages. A **service** is a request with
one response. An **action** is a longer operation that can report progress and
be cancelled.

This computer uses ROS 2 Lyrical:

```bash
# Make ROS 2 Lyrical commands available in the current terminal.
source /opt/ros/lyrical/setup.bash
```

Sourcing affects only the current terminal.

## Build and Source a Workspace

Run from the repository root:

```bash
# Load the base ROS 2 installation.
source /opt/ros/lyrical/setup.bash

# Build all packages in the current workspace.
colcon build --symlink-install

# Add this workspace's built packages to the current terminal.
source install/setup.bash
```

```bash
# Build only one package when working on a focused change.
colcon build --symlink-install --packages-select px4_sitl_bringup

# List packages that colcon can discover in the source tree.
colcon list
```

The PX4 Docker runner can be used directly without building ROS launch support.
The ROS launch command requires a successful workspace build and
`source install/setup.bash`.

## Nodes

```bash
# List currently discovered ROS 2 nodes.
ros2 node list

# Show publishers, subscribers, services, and actions for one node.
ros2 node info /node_name
```

If a node does not appear, check that every terminal has sourced ROS 2 and that
the processes use the same `ROS_DOMAIN_ID`.

## Topics

```bash
# List topic names.
ros2 topic list

# List topic names with their message types.
ros2 topic list -t

# Show publisher and subscriber counts.
ros2 topic info /topic_name

# Show detailed endpoint and QoS information.
ros2 topic info /topic_name --verbose

# Print one message and then exit.
ros2 topic echo /topic_name --once

# Continuously print messages until Ctrl+C is pressed.
ros2 topic echo /topic_name

# Estimate the publishing frequency.
ros2 topic hz /topic_name

# Estimate message bandwidth.
ros2 topic bw /topic_name
```

Use `ros2 interface show` to understand a message's fields:

```bash
# Display the structure of an IMU message.
ros2 interface show sensor_msgs/msg/Imu
```

Publishing manually changes runtime behavior. Review the message carefully:

```bash
# Example syntax only: publish a String message once.
ros2 topic pub --once /example std_msgs/msg/String "{data: hello}"
```

## Services

```bash
# List services and their types.
ros2 service list -t

# Show the request and response structure.
ros2 interface show example_interfaces/srv/AddTwoInts

# Display the type used by one service.
ros2 service type /service_name
```

Calling a service can change a running system:

```bash
# Example syntax only.
ros2 service call /service_name package_name/srv/ServiceType "{field: value}"
```

## Actions

```bash
# List actions and their types.
ros2 action list -t

# Show information about one action.
ros2 action info /action_name

# Show the goal, result, and feedback fields of an action type.
ros2 interface show package_name/action/ActionType
```

Sending a goal changes runtime behavior:

```bash
# Example syntax only.
ros2 action send_goal /action_name package_name/action/ActionType "{field: value}"
```

## Parameters

```bash
# List every node and its declared parameters.
ros2 param list

# List parameters owned by one node.
ros2 param list /node_name

# Read one parameter.
ros2 param get /node_name parameter_name

# Save a node's parameters to YAML.
ros2 param dump /node_name
```

Changing a parameter can immediately change node behavior:

```bash
# Example syntax only.
ros2 param set /node_name parameter_name value
```

## Launch Files

```bash
# List launch arguments without starting the launch system.
ros2 launch package_name file.launch.py --show-args

# Launch the project's PX4 runner through ROS 2.
ros2 launch px4_sitl_bringup px4_sitl.launch.py

# Override a declared launch argument.
ros2 launch px4_sitl_bringup px4_sitl.launch.py headless:=1
```

## TF Transforms

TF describes where coordinate frames are relative to one another:

```text
world → vehicle body → sensor
```

```bash
# Continuously print the transform between two frames.
ros2 run tf2_ros tf2_echo world base_link

# Generate a PDF showing the discovered TF tree.
ros2 run tf2_tools view_frames

# Inspect dynamic transforms.
ros2 topic echo /tf --once

# Inspect fixed transforms.
ros2 topic echo /tf_static --once
```

Frame names depend on the active robot and branch.

## ROS Bags

A ROS bag records topic messages for later inspection and replay.

```bash
# Confirm that rosbag commands are installed.
ros2 bag --help

# Record selected X3 telemetry topics.
# Put -o before the topic list with the installed Lyrical CLI.
ros2 bag record \
  -o bags/telemetry_test_01 \
  /tf \
  /tf_static \
  /laser_scan \
  /x3_lidar/imu \
  /x3_lidar/range/down

# Show bag metadata without replaying it.
ros2 bag info bags/telemetry_test_01

# Replay the recorded messages into the active ROS graph.
ros2 bag play bags/telemetry_test_01
```

The X3 telemetry topics belong to `feature/telemetry-sensors`. PX4 ROS 2 topics
will be documented after the Micro XRCE-DDS checkpoint.

## Environment Checks

```bash
# Display the ROS distribution sourced in this terminal.
printf '%s\n' "$ROS_DISTRO"

# Display the ROS domain used for discovery.
printf '%s\n' "${ROS_DOMAIN_ID:-0}"

# Show installed ROS environment paths.
printf '%s\n' "$AMENT_PREFIX_PATH"
```

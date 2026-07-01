# ROS 2 + Gazebo System Graph Tool

`generate_system_graph.py` is a read-only developer/debugging tool that draws a snapshot of the live ROS 2 + Gazebo communication graph.

It has two views:

- `presentation`: a curated architecture map for this drone project.
- `debug`: a detailed graph with the raw ROS/Gazebo inspection results.

It can show:

- Gazebo Transport topics from `gz topic -l`
- bridge mappings from this repo's `ros_gz_bridge` YAML files
- ROS 2 topics from `ros2 topic list -t`
- ROS 2 nodes and publisher/subscriber relationships from `ros2 node info`
- ROS 2 services and actions when the CLI can inspect them

It does not launch the drone, change any bridge, command motors, modify controller behavior, or replace tools like Foxglove or `rqt_graph`.

## Install Graphviz

```bash
sudo apt install graphviz
```

The script still writes the DOT file if Graphviz is not installed; only SVG rendering is skipped.

## Run

Start the base simulation first, and start the controller in another terminal if you want controller nodes and motor topics to appear.

```bash
cd ~/LIDAR_mapping_drone
source /opt/ros/lyrical/setup.bash
source install/setup.bash

ros2 launch lidar_mapping_drone_bringup lidar_mapping_drone.launch.py
```

In another terminal:

```bash
cd ~/LIDAR_mapping_drone
source /opt/ros/lyrical/setup.bash
source install/setup.bash

python3 tools/generate_system_graph.py --view presentation
xdg-open tools/generated/ros_gz_system_graph.svg
```

The default `presentation` outputs are:

```text
tools/generated/ros_gz_system_graph.dot
tools/generated/ros_gz_system_graph.svg
```

For the detailed graph:

```bash
python3 tools/generate_system_graph.py --view debug
xdg-open tools/generated/ros_gz_system_graph_debug.svg
```

The `debug` outputs are:

```text
tools/generated/ros_gz_system_graph_debug.dot
tools/generated/ros_gz_system_graph_debug.svg
```

Useful options:

```bash
python3 tools/generate_system_graph.py --view presentation --include-services
python3 tools/generate_system_graph.py --view presentation --include-gazebo-internal
python3 tools/generate_system_graph.py --view presentation --include-ros-internal
python3 tools/generate_system_graph.py --output-dir /tmp/system_graph
```

## Presentation View

The presentation view hides common ROS 2 and Gazebo boilerplate by default:

- ROS parameter services
- `/rosout`
- `/parameter_events`
- internal transform listener helper nodes
- Gazebo GUI topics
- Gazebo world scene/stats/light configuration topics

It keeps the main project architecture visible:

- Gazebo `/lidar2` -> `ros_gz_bridge` -> ROS `/laser_scan`
- Gazebo `/model/x3_lidar/pose` -> `ros_gz_bridge` -> ROS `/tf`
- ROS `/X3/gazebo/command/motor_speed` -> `ros_gz_bridge` -> Gazebo `/X3/gazebo/command/motor_speed`
- running controller, keyboard, RViz, and static transform nodes when present

The red, thicker edges highlight the controller feedback path where the graph can infer it.
Topic labels keep full message type names and render topic name/type on separate lines. The graph title includes a compact legend for the shapes and colors used in the current view.

## Limitations

- ROS 2 graph inspection is cleaner than Gazebo Transport graph inspection.
- Gazebo topics may not expose publisher/subscriber relationships as cleanly as ROS 2 nodes.
- Bridge relationships are best understood from the project YAML files.
- If Gazebo, ROS 2, or a topic disappears while inspecting, the script keeps going and records a warning.
- The generated diagram is a debugging and learning aid, not a formal proof of the complete runtime system.
- The system graph shows `/tf` as a communication topic; it does not draw a full TF frame tree.

## Bridge YAML Assumptions

The parser expects the current project format: a top-level list of bridge entries where each entry has scalar fields such as `ros_topic_name`, `gz_topic_name`, `ros_type_name`, `gz_type_name`, and `direction`.

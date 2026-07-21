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
- Gazebo `/x3_lidar/imu` -> `ros_gz_bridge` -> ROS `/x3_lidar/imu`
- Gazebo `/x3_lidar/range/down` -> `ros_gz_bridge` -> ROS `/x3_lidar/range/down`
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

# Telemetry vs TF Plot Tool

`plot_telemetry_vs_tf.py` turns a recorded telemetry bag into an offline sensor
comparison dashboard. It treats the recorded Gazebo pose on `/tf` as ground
truth and compares it with:

- IMU orientation and angular velocity on `/x3_lidar/imu`
- downward range on `/x3_lidar/range/down`

The range comparison reconstructs the range sensor's recorded TF pose and
computes the expected intersection between its beam and the horizontal ground
plane. This is more accurate than treating raw beam range as if it were the
drone model's world-frame altitude.

The tool is read-only. It does not replay the bag, publish ROS topics, launch
the simulation, or change the controller.

## Plot a Bag

Source ROS 2 and the workspace, then provide a rosbag directory:

```bash
cd ~/LIDAR_mapping_drone
source /opt/ros/lyrical/setup.bash
source install/setup.bash

python3 tools/plot_telemetry_vs_tf.py \
  bags/telemetry_sensors_20260705_160549
```

The default output is written inside the selected bag:

```text
bags/telemetry_sensors_20260705_160549/analysis/telemetry_vs_tf.png
```

Because the `bags/` directory is ignored by Git, generated analysis images are
not committed.

Open an interactive Matplotlib window as well as saving the image:

```bash
python3 tools/plot_telemetry_vs_tf.py BAG_DIRECTORY --show
```

Choose a different image path or format:

```bash
python3 tools/plot_telemetry_vs_tf.py BAG_DIRECTORY \
  --output /tmp/telemetry_comparison.svg
```

If the simulated ground plane is not at world `z = 0`, provide its height:

```bash
python3 tools/plot_telemetry_vs_tf.py BAG_DIRECTORY --ground-z 0.25
```

The dashboard includes:

- measured downward range, TF-derived expected beam range, and model TF `z`
- range residual error
- IMU and TF roll, pitch, and unwrapped yaw
- IMU angular velocity and angular rates derived from TF
- IMU linear acceleration
- IMU orientation error against interpolated TF orientation
- TF ground-truth XY path
- sample rates, valid range count, and RMS comparison errors

The plot is only as complete as the bag. Missing IMU or range topics are marked
in the dashboard, while a missing `x3_lidar` pose transform stops the analysis
because there is no ground-truth reference.

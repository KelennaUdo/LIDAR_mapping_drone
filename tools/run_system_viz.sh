  cd ~/LIDAR_mapping_drone
  source /opt/ros/lyrical/setup.bash
  source install/setup.bash

  python3 tools/generate_system_graph.py --view presentation
  xdg-open tools/generated/ros_gz_system_graph.svg

  # python3 tools/generate_system_graph.py --view debug
  # xdg-open tools/generated/ros_gz_system_graph_debug.svg

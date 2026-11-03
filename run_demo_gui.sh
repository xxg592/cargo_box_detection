#!/usr/bin/env bash
# Start the whole system with the Gazebo 3D window.
#
#   ./run_demo_gui.sh              # normal run, mission starts automatically
#   ./run_demo_gui.sh start_mission:=False   # simulator only, no mission
#
# Note: closing the Gazebo window ends the simulation. Run this script again to
# bring everything back; nothing is corrupted.
set -e
cd "$(dirname "$0")"

if [ ! -f ros2_ws/install/setup.bash ]; then
  echo "workspace not built yet, building now..."
  source /opt/ros/jazzy/setup.bash
  (cd ros2_ws && colcon build --symlink-install --packages-select cargo_box_detector)
fi

source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

echo "starting the demo with the 3D window; close the window to stop it"
exec ros2 launch cargo_box_detector cargo_box_system.launch.py headless:=False "$@"

"""Single entry point for the cargo-box warehouse demo.

Starts, in this order and exactly once:
  Gazebo Sim -> cargo_robot spawn -> ros_gz bridge -> robot/camera TF
  -> RGB-D point cloud to /scan -> static map -> Nav2 stack
  -> detector / fusion / FSM / navigation coordinator / metrics nodes

There is deliberately no AMCL and no SLAM here: the warehouse map is a
static rasterisation of the Gazebo world and the Gazebo diff-drive
odometry starts at the spawn pose, so map -> odom is an exact constant
transform.  That removes the map/odom TF time-extrapolation failures
that blocked the previous Nav2 runs.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    package_share = get_package_share_directory("cargo_box_detector")
    project_root = os.path.expanduser("~/cargo_box_detection")

    bridge_config = os.path.join(package_share, "config", "bridge_config.yaml")
    p2l_config = os.path.join(
        package_share, "config", "pointcloud_to_laserscan.yaml"
    )
    map_yaml = os.path.join(package_share, "config", "warehouse_static.yaml")
    params_file = os.path.join(package_share, "config", "nav2_params.yaml")
    models_dir = os.path.join(project_root, "simulation", "models")
    model_file = os.path.join(models_dir, "cargo_robot", "model.sdf")
    world_file = os.path.join(
        project_root, "simulation", "worlds", "cargo_world.sdf"
    )

    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")
    start_sim = LaunchConfiguration("start_sim")
    start_mission = LaunchConfiguration("start_mission")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_yaw = LaunchConfiguration("spawn_yaw")

    mission_enabled = IfCondition(
        PythonExpression(["'", start_mission, "' == 'True'"])
    )

    # ---------------------------------------------------------------- Gazebo
    set_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        models_dir + ":" + os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
    )

    sim_enabled = PythonExpression(["'", start_sim, "' == 'True'"])

    gazebo_server = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-s", "-v", "3", world],
        output="screen",
        condition=IfCondition(
            PythonExpression([
                "'", start_sim, "' == 'True' and '", headless, "' == 'True'"
            ])
        ),
    )

    gazebo_gui = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-v", "3", world],
        output="screen",
        condition=IfCondition(
            PythonExpression([
                "'", start_sim, "' == 'True' and '", headless, "' != 'True'"
            ])
        ),
    )

    spawn_robot = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=[
                    "-world", "cargo_world",
                    "-file", model_file,
                    "-name", "cargo_robot",
                    "-x", spawn_x,
                    "-y", spawn_y,
                    "-z", "0.0",
                    "-Y", spawn_yaw,
                ],
                condition=IfCondition(sim_enabled),
            )
        ],
    )

    # ---------------------------------------------------------------- Bridge
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        parameters=[{"config_file": bridge_config, "use_sim_time": True}],
    )

    # ------------------------------------------------------------------- TF
    # The Gazebo diff-drive plugin only publishes odom -> base_link.
    # map -> odom is the (known, constant) spawn pose, because odometry
    # integrates from the spawn point and the map is world-aligned.
    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_to_odom",
        output="screen",
        arguments=[
            "--x", spawn_x, "--y", spawn_y, "--z", "0.0",
            "--roll", "0.0", "--pitch", "0.0", "--yaw", spawn_yaw,
            "--frame-id", "map", "--child-frame-id", "odom",
        ],
    )

    # Sensor frames from model.sdf: pose (0.22, 0, 0.16) with pitch 0.20 rad.
    camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_front_camera_tf",
        output="screen",
        arguments=[
            "--x", "0.22", "--y", "0.0", "--z", "0.16",
            "--roll", "0.0", "--pitch", "0.20", "--yaw", "0.0",
            "--frame-id", "base_link",
            "--child-frame-id", "cargo_robot/base_link/front_camera",
        ],
    )

    depth_camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_depth_camera_tf",
        output="screen",
        arguments=[
            "--x", "0.22", "--y", "0.0", "--z", "0.16",
            "--roll", "0.0", "--pitch", "0.20", "--yaw", "0.0",
            "--frame-id", "base_link",
            "--child-frame-id", "cargo_robot/base_link/front_depth_camera",
        ],
    )

    # --------------------------------------------------------- LaserScan
    pointcloud_to_laserscan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        parameters=[p2l_config, {"use_sim_time": True}],
        remappings=[
            ("cloud_in", "/cargo_robot/depth_camera/points"),
            ("scan", "/scan"),
        ],
    )

    # --------------------------------------------------------------- Nav2
    nav2_parameters = [params_file, {"use_sim_time": True}]

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{
            "yaml_filename": map_yaml,
            "use_sim_time": True,
            "frame_id": "map",
            "topic_name": "map",
        }],
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=nav2_parameters,
    )

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=nav2_parameters,
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=nav2_parameters,
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=nav2_parameters,
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "autostart": True,
            # 0.0 disables bond heartbeats: under a low real-time factor the
            # default 10 s bond timeout fires and tears the stack down.
            "bond_timeout": 0.0,
            "node_names": [
                "map_server",
                "planner_server",
                "controller_server",
                "behavior_server",
                "bt_navigator",
            ],
        }],
    )

    # ------------------------------------------------------- Mission nodes
    detector = Node(
        package="cargo_box_detector",
        executable="cargo_box_detector_node",
        name="cargo_box_detector_node",
        output="screen",
        parameters=[{"image_topic": "/cargo_robot/front_camera"}],
    )

    fusion = Node(
        package="cargo_box_detector",
        executable="cargo_position_fusion_node",
        name="cargo_position_fusion_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    fsm = Node(
        package="cargo_box_detector",
        executable="cargo_delivery_fsm_node",
        name="cargo_delivery_fsm_node",
        output="screen",
    )

    navigation = Node(
        package="cargo_box_detector",
        executable="cargo_navigation_node",
        name="cargo_navigation_node",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "goal_frame": "map",
            "approach_distance": 1.05,
            "dropoff_x": 3.5,
            "dropoff_y": 0.0,
            "dropoff_yaw": 0.0,
            "pickup_delay_s": 2.0,
            "dropoff_delay_s": 2.0,
        }],
    )

    metrics = Node(
        package="cargo_box_detector",
        executable="cargo_metrics_node",
        name="cargo_metrics_node",
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "world", default_value=world_file,
            description="Gazebo world file",
        ),
        DeclareLaunchArgument(
            "headless", default_value="True",
            description="Run Gazebo without the 3D GUI",
        ),
        DeclareLaunchArgument(
            "start_sim", default_value="True",
            description="Start Gazebo and spawn the robot",
        ),
        DeclareLaunchArgument(
            "spawn_x", default_value="2.5",
            description="Robot spawn x in map frame",
        ),
        DeclareLaunchArgument(
            "spawn_y", default_value="0.0",
            description="Robot spawn y in map frame",
        ),
        DeclareLaunchArgument(
            "spawn_yaw", default_value="0.0",
            description="Robot spawn yaw in map frame (rad)",
        ),
        DeclareLaunchArgument(
            "start_mission", default_value="True",
            description=(
                "Start detector / fusion / FSM / navigation / metrics. "
                "Set to False for a pure Nav2 navigation test."
            ),
        ),

        set_resource_path,

        # Simulation
        gazebo_server,
        gazebo_gui,
        spawn_robot,
        bridge,

        # Frames, sensors
        map_to_odom,
        camera_tf,
        depth_camera_tf,
        pointcloud_to_laserscan,

        # Navigation, started after the robot exists so that the costmaps find
        # map -> odom -> base_link on their first activation attempt.
        TimerAction(
            period=12.0,
            actions=[
                map_server,
                planner_server,
                controller_server,
                behavior_server,
                bt_navigator,
                lifecycle_manager,
            ],
        ),

        # Mission, started once Nav2 is active: the FSM reacts to the first
        # detection, and a goal sent before bt_navigator finishes activating
        # is rejected.
        TimerAction(
            period=20.0,
            actions=[
                GroupAction(
                    condition=mission_enabled,
                    actions=[detector, fusion, fsm, navigation, metrics],
                ),
            ],
        ),
    ])
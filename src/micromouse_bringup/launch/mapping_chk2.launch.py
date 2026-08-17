import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_bringup = get_package_share_directory("micromouse_bringup")
    pkg_desc = get_package_share_directory("micromouse_description")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    world_path = os.path.join(pkg_bringup, "worlds", "micromouse.sdf")
    maze_path = os.path.join(pkg_bringup, "worlds", "maze.sdf")
    xacro_path = os.path.join(pkg_desc, "urdf", "micromouse.urdf.xacro")
    bridge_config = os.path.join(pkg_bringup, "config", "bridge.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_description = ParameterValue(Command(["xacro ", xacro_path]), value_type=str)

    # --- simulacija ---
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": [world_path, " -r -v 4"]}.items(),
    )
    spawn_maze = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-file",
            maze_path,
            "-name",
            "maze",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0",
        ],
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": use_sim_time}
        ],
    )
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "micromouse",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.03",
            "-Y",
            "1.5708",
        ],
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[{"config_file": bridge_config, "use_sim_time": use_sim_time}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
    )

    # --- mapiranje + prikaz ---
    mapping = Node(
        package="micromouse_mapping", executable="mapping_node", output="screen"
    )
    viz = Node(
        package="micromouse_mapping", executable="maze_viz_node", output="screen"
    )
    planner = Node(
        package="micromouse_mapping", executable="planner_node", output="screen"
    )

    # --- staticki transform da 'map' postoji (makne crvenu gresku u RViz-u) ---
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "map"],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            gz_sim,
            spawn_maze,
            robot_state_publisher,
            spawn_robot,
            bridge,
            mapping,
            viz,
            planner,
            static_tf,
            rviz,
        ]
    )

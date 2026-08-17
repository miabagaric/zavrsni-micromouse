import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_bringup = get_package_share_directory('micromouse_bringup')
    pkg_desc = get_package_share_directory('micromouse_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_path = os.path.join(pkg_bringup, 'worlds', 'micromouse.sdf')
    xacro_path = os.path.join(pkg_desc, 'urdf', 'micromouse.urdf.xacro')
    bridge_config = os.path.join(pkg_bringup, 'config', 'bridge.yaml')
    maze_path = os.path.join(pkg_bringup, 'worlds', 'maze.sdf')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # robot_description iz xacro-a (obradjuje se u trenutku pokretanja)
    robot_description = ParameterValue(
        Command(['xacro ', xacro_path]),
        value_type=str
    )

    # 1) Pokreni Gazebo Harmonic s nasim svijetom (-r = odmah krece, -v 4 = verbose)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': [world_path, ' -r -v 4']}.items()
    )

    # 2) robot_state_publisher: objavljuje robot_description + TF iz URDF-a
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    # 3) Spawn robota u Gazebo iz robot_description topica
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'micromouse',
            '-x', '0.0', '-y', '0.0', '-z', '0.03',
            '-Y', '1.5708',   # yaw +90 deg -> robot gleda prema +y (sjever)
        ],
    )

    # 4) Most ROS <-> Gazebo (cmd_vel, odom, tf, joint_states, clock)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_config,
            'use_sim_time': use_sim_time,
        }],
    )

    # Spawn labirinta (staticki model) u svijet
    spawn_maze = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-file', maze_path, '-name', 'maze', '-x', '0', '-y', '0', '-z', '0'],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gz_sim,
        spawn_maze,
        robot_state_publisher,
        spawn,
        bridge,
    ])

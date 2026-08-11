import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    line_tracer_share = get_package_share_directory('line_tracer')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    world_path = os.path.join(line_tracer_share, 'worlds', 'line_track.world')

    # Gazebo Classic 서버+클라이언트 실행
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    xacro_path = os.path.join(line_tracer_share, 'urdf', 'simple_bot.urdf.xacro')
    robot_description = xacro.process_file(xacro_path).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    spawn_robot = Node(
       package='gazebo_ros',
       executable='spawn_entity.py',
       arguments=['-topic', 'robot_description', '-entity', 'simple_bot',
               '-x', '0', '-y', '-1', '-z', '0.1'],
       output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
    ])
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import ExecuteProcess

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'cpp_pubsub'
    pkg_share = get_package_share_directory(pkg_name)
    
    urdf_file = os.path.join(pkg_share, 'urdf', 'drone.xacro')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'default_view.rviz')
    world_file = os.path.join(pkg_share, 'worlds', 'empty_sky.world')

    # 1. Robot State Publisher (TF2)
    # FIX: Use 'Command' to run the xacro compiler and generate pure URDF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        # THIS IS THE FIX: Wrapping the Command in ParameterValue and forcing value_type=str
        parameters=[{'robot_description': ParameterValue(Command(['xacro ', urdf_file]), value_type=str)}]
    )

    # 2. RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    # 3. Gazebo Server & Client (Direct Execution)
    # -s libgazebo_ros_init.so : Connects Gazebo to the ROS 2 network
    # -s libgazebo_ros_factory.so : Allows spawn_entity.py to work
    # -u : Starts Gazebo paused
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            '-u'
        ],
        output='screen'
    )

    # 4. Spawn the Drone in Gazebo
    # Spawn it higher so gravity is obvious before it settles on the ground.
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'my_drone', '-topic', 'robot_description', '-z', '15.0'],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        gazebo,
        spawn_entity,
        rviz_node
    ])
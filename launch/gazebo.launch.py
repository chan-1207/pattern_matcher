from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # # Turtlebot
    # turtlebot3_bringup_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('turtlebot3_bringup'),
    #             'launch',
    #             'robot.launch.py'
    #         ])
    #     ])
    # )

    turtlebot3_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),
                'launch',
                'turtlebot3_world.launch.py'
            ])
        ])
    )

    # Pattern Model
    urdf_path = os.path.join(
        get_package_share_directory('pattern_matcher'),
        'urdf',
        'pattern.urdf'
    )

    pattern_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='pattern_state_publisher',
        parameters=[{
            'robot_description': Command(['cat ', urdf_path])
        }],
        remappings=[
            ('robot_description', 'pattern_description'),
            ('/joint_states', '/pattern_joint_states')
        ]
    )

    pattern_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='pattern_joint_state_publisher',
        parameters=[{
            'robot_description': Command(['cat ', urdf_path])
        }],
        remappings=[
            ('robot_description', 'pattern_description'),
            ('/joint_states', '/pattern_joint_states')
        ]
    )

    pattern_spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='pattern_spawn_robot',
        arguments=[
            '-entity', 'pattern',
            '-file', urdf_path,
            '-x', '-0.5',
            '-y', '-0.5',
            '-z', '0.25',
            '-Y', '-1.5707'
        ]
    )

    return LaunchDescription([
        # turtlebot3_bringup_launch,
        turtlebot3_gazebo_launch,
        pattern_state_publisher,
        pattern_joint_state_publisher,
        pattern_spawn
    ])

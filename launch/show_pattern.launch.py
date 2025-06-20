from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = FindPackageShare('pattern_matcher').find('pattern_matcher')
    
    urdf_path = os.path.join(pkg_share, 'urdf', 'pattern.urdf')
    rviz_config = os.path.join(pkg_share, 'config', 'pattern.rviz')

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

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        pattern_state_publisher,
        pattern_joint_state_publisher,
        rviz_node
    ])
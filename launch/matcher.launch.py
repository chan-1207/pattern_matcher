from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pattern_matcher_node = Node(
        package='pattern_matcher',
        executable='pattern_matcher',
        name='pattern_matcher',
        output='screen',
        parameters=[{
            'scan_topic': 'scan',
            'pattern_topic': 'pattern',
            'clustered_points_topic': 'clustered_points',
            'pattern_filepath': ''
        }]
    )

    # Include turtlebot3_bringup's rviz2.launch.py
    turtlebot3_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_bringup'),
                'launch',
                'rviz2.launch.py'
            ])
        ])
    )

    return LaunchDescription([
        pattern_matcher_node,
        turtlebot3_rviz_launch
    ])
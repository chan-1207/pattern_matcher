from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('pattern_matcher')
    rviz_config = os.path.join(pkg_dir, 'config', 'pattern_visualization.rviz')
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'target_frame',
            default_value='front_of_station',
            description='Target frame to navigate to'
        ),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_link',
            description='Base frame of the robot'
        ),
        DeclareLaunchArgument(
            'map_frame',
            default_value='map',
            description='Map frame'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Whether to launch RViz'
        ),
        
        # Pattern matcher node
        Node(
            package='pattern_matcher',
            executable='pattern_matcher',
            name='pattern_matcher',
            parameters=[{
                'scan_topic': 'scan',
                'pattern_topic': 'pattern',
                'clustered_points_topic': 'clustered_points',
                'pattern_filepath': ''
            }],
            output='screen'
        ),
        
        # Navigator node
        Node(
            package='pattern_matcher',
            executable='navigator',
            name='navigator',
            parameters=[{
                'target_frame': LaunchConfiguration('target_frame'),
                'base_frame': LaunchConfiguration('base_frame'),
                'map_frame': LaunchConfiguration('map_frame')
            }],
            output='screen'
        ),
        
        # RViz node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            condition=LaunchConfiguration('use_rviz'),
            output='screen'
        )
    ]) 
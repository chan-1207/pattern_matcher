from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
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
        
        # Pattern matcher node
        # Node(
        #     package='pattern_matcher',
        #     executable='pattern_matcher',
        #     name='pattern_matcher',
        #     parameters=[{
        #         'scan_topic': 'scan',
        #         'pattern_topic': 'pattern',
        #         'clustered_points_topic': 'clustered_points',
        #         'pattern_filepath': ''
        #     }],
        #     output='screen'
        # ),
        
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
        )
    ]) 
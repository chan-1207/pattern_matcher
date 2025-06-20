from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pattern_recorder_node = Node(
        package='pattern_matcher',
        executable='pattern_recorder',
        name='pattern_recorder',
        output='screen',
        parameters=[{
            'mean_queue_size': 20,
            'location_filter_x': 0.5,
            'location_filter_y': 0.2,
            'voxel_leaf_size': 0.0075,
            'scan_topic': 'scan',
            'output_file': ''
        }]
    )

    return LaunchDescription([
        pattern_recorder_node
    ])
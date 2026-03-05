import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    vins_estimator_dir = get_package_share_directory('vins_estimator')
    
    # Declare arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(vins_estimator_dir, 'config', 'euroc', 'euroc_config.yaml'),
        description='Path to the config file'
    )
    
    # Feature Tracker Node
    feature_tracker_node = Node(
        package='feature_tracker',
        executable='feature_tracker_node',
        name='feature_tracker',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
            'vins_folder': vins_estimator_dir + '/'
        }]
    )
    
    # VINS Estimator Node
    vins_estimator_node = Node(
        package='vins_estimator',
        executable='vins_estimator_node',
        name='vins_estimator',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
            'vins_folder': vins_estimator_dir + '/'
        }]
    )
    
    # Pose Graph Node
    pose_graph_node = Node(
        package='pose_graph',
        executable='pose_graph_node',
        name='pose_graph',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
            'visualization_shift_x': 0,
            'visualization_shift_y': 0,
            'skip_cnt': 0,
            'skip_dis': 0.0
        }]
    )

    return LaunchDescription([
        config_file_arg,
        feature_tracker_node,
        vins_estimator_node,
        pose_graph_node
    ])

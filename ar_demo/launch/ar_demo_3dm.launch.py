import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    vins_estimator_dir = get_package_share_directory('vins_estimator')
    ar_demo_dir = get_package_share_directory('ar_demo')
    
    # Declare arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(vins_estimator_dir, 'config', '3dm', '3dm_config.yaml'),
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
    
    # AR Demo Node
    ar_demo_node = Node(
        package='ar_demo',
        executable='ar_demo_node',
        name='ar_demo_node',
        output='screen',
        remappings=[
            ('~/image_raw', '/mv_25001498/image_raw'),
            ('~/camera_pose', '/vins_estimator/camera_pose'),
            ('~/pointcloud', '/vins_estimator/point_cloud'),
        ],
        parameters=[{
            'calib_file': LaunchConfiguration('config_file'),
            'use_undistored_img': False
        }]
    )

    return LaunchDescription([
        config_file_arg,
        feature_tracker_node,
        vins_estimator_node,
        ar_demo_node
    ])

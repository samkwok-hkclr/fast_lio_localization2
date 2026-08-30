from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

# Import ComposableNode modules
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_path = get_package_share_directory("fast_lio_localization2")
    default_config_path = os.path.join(package_path, "config")
    default_rviz_config_path = os.path.join(package_path, "rviz", "fastlio_localization.rviz")
    default_map_path = os.path.join(package_path, "map", "map_v3_0.05m.pcd")

    use_sim_time = LaunchConfiguration("use_sim_time")
    config_path = LaunchConfiguration("config_path")
    config_file = LaunchConfiguration("config_file")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_cfg = LaunchConfiguration("rviz_cfg")
    pcd_map_topic = LaunchConfiguration("pcd_map_topic")
    pcd_map_path = LaunchConfiguration("map")

    # Declare arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", 
        default_value="false", 
        description="Use simulation (Gazebo) clock if true"
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        "config_path", 
        default_value=default_config_path, 
        description="Yaml config file path"
    )
    declare_config_file_cmd = DeclareLaunchArgument(
        "config_file", 
        default_value="mid360.yaml", 
        description="Config file"
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", 
        default_value="false", 
        description="Use RViz to monitor results"
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        "rviz_cfg", 
        default_value=default_rviz_config_path, 
        description="RViz config file path"
    )
    declare_map_path = DeclareLaunchArgument(
        "map", 
        default_value=default_map_path, 
        description="Path to PCD map file"
    )
    declare_pcd_map_topic = DeclareLaunchArgument(
        "pcd_map_topic", 
        default_value="/map", 
        description="Topic to publish PCD map"
    )
    
    # Load parameters from yaml file
    fast_lio_node = Node(
        package="fast_lio_localization2",
        executable="fastlio_mapping",
        parameters=[
            PathJoinSubstitution([config_path, config_file]), 
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )
    
    # Global localization and Transform Fusion running in the SAME container
    localization_container = ComposableNodeContainer(
        name='localization_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # 1. The Global Localization Component
            ComposableNode(
                package='fast_lio_localization2',
                plugin='fast_lio_localization::FastLIOLocalization',
                name='global_localization',
                parameters=[
                    PathJoinSubstitution([config_path, config_file]), # Load all params from YAML
                    {"pcd_map_path": pcd_map_path}                    # Allow command-line 'map:=' override
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            # 2. The Transform Fusion Component
            ComposableNode(
                package='fast_lio_localization2',
                plugin='fast_lio_localization::TransformFusion',
                name='transform_fusion',
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ],
        output='screen',
    )
    
    # PCD to PointCloud2 publisher
    pcd_publisher_node = Node(
        package="pcl_ros",
        executable="pcd_to_pointcloud",
        name="map_publisher",
        output="screen",
        parameters=[{"file_name": pcd_map_path,
                     "tf_frame": "map",
                     "interval": 1.0}], # Updated period_ms_ to interval for ROS 2
        remappings=[
            ("cloud_pcd", pcd_map_topic),
        ]
    )

    rviz_node = Node(
        package="rviz2", 
        executable="rviz2",
        arguments=["-d", rviz_cfg], 
        condition=IfCondition(use_rviz)
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    ld.add_action(declare_map_path)
    ld.add_action(declare_pcd_map_topic)

    ld.add_action(fast_lio_node)
    ld.add_action(rviz_node)
    ld.add_action(localization_container)
    ld.add_action(pcd_publisher_node)

    return ld
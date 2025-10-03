import dwm1001_visualization
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument 
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description() -> LaunchDescription:

    dwm_config_val = LaunchConfiguration('dwm_config')
    default_dwm_config_path = PathJoinSubstitution([FindPackageShare('rover_launch'),
                                                     'config',
                                                     'anchors.yaml'])
    dwm_config_launch_arg = DeclareLaunchArgument(
        'dwm_config',
        default_value=default_dwm_config_path,
        description='Configuration file for the anchors'
    )

    dwm1001_driver = Node(
        package="dwm1001_visualization",
        executable="anchor_visualizer",
        parameters=[PathJoinSubstitution([
    	FindPackageShare('rover_launch'),
    	'config',
    	'anchors.yaml'
])]


    )

    return LaunchDescription(
        [dwm_config_launch_arg, dwm1001_driver]
    )

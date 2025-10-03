from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # Percorso al file pix_rover_driver.launch.xml
    pix_rover_launch = os.path.join(
        get_package_share_directory('pix_rover_driver'),
        'launch',
        'pix_rover_driver.launch.xml'
    )

    # Percorso al file socket_can_bridge.launch.xml
    socket_can_bridge_launch = os.path.join(
        get_package_share_directory('ros2_socketcan'),
        'launch',
        'socket_can_bridge.launch.xml'
    )

    # Percorso al file active_node.launch.py del pacchetto dwm1001_launch
    dwm1001_launch = os.path.join(
        get_package_share_directory('dwm1001_launch'),
        'launch',
        'active_node.launch.py'
    )

    #  Percorso al file anchors.launch.py
    anchors_launch = os.path.join(
        get_package_share_directory('rover_launch'),
        'launch',
        'anchors.launch.py'
    )

    return LaunchDescription([
        # Nodo reference_node
        Node(
            package='reference_node',
            executable='reference_node',
            name='reference_node',
            output='screen'
        ),

        # Include file socket_can_bridge.launch.xml
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(socket_can_bridge_launch)
        ),

        # Include file pix_rover_driver.launch.xml
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(pix_rover_launch)
        ),

        # Include file active_node.launch.py
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(dwm1001_launch)	
        ),

        # Include file anchors.launch.py
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(anchors_launch)
        ),
    ])


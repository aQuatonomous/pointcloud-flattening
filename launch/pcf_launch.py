from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Launch lidar processing module
    pointcloud_flattening = Node(
        package='pointcloud_flattening',
        executable='Flatten',
    )

    # Package launch description actions 
    ld.add_action(pointcloud_flattening)

    return ld

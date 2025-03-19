import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Get the share directory of your package (change 'line_following' to your package name if different)
    pkg_share = get_package_share_directory('line_following')
    # Build the full path to your custom world file
    world_file = os.path.join(pkg_share, 'worlds', 'world')
    
    # Create a process to launch Gazebo with your world file
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    
    ld = LaunchDescription()
    ld.add_action(gazebo)
    return ld

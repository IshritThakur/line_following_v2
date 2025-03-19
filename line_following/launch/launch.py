import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Get the share directory of your package
    pkg_share = get_package_share_directory('line_following')
    # Construct the full path to your world file (adjust folder/file names as needed)
    world_file = os.path.join(pkg_share, 'world', 'world.xml')
    
    # Launch Gazebo with your custom world file
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    
    ld = LaunchDescription()
    ld.add_action(gazebo)
    return ld

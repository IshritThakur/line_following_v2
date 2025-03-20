import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package share directory (assumes your world file is installed there)
    pkg_share = get_package_share_directory('line_following')
    # Use your custom world file from the installed share directory
    world_file = os.path.join(pkg_share, 'world', 'world.xml')
    
    # Launch Gazebo with the custom world file
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    
    # Launch the controller node (make sure your entry point is correct)
    controller_node = Node(
        package='line_following',
        executable='controller',
        output='screen'
    )
    
    ld = LaunchDescription()
    ld.add_action(gazebo)
    ld.add_action(controller_node)
    return ld

# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import ExecuteProcess
# from launch_ros.actions import Node

# def generate_launch_description():
#     # Get the package share directory (assumes your world file is installed there)
#     pkg_share = get_package_share_directory('line_following')
#     # Use your custom world file from the installed share directory
#     world_file = os.path.join(pkg_share, 'world', 'world.xml')
    
#     # Launch Gazebo with the custom world file
#     gazebo = ExecuteProcess(
#         cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
#         output='screen'
#     )
    
#     # Launch the controller node (make sure your entry point is correct)
#     controller_node = Node(
#         package='line_following',
#         executable='controller',
#         output='screen'
#     )
    
#     ld = LaunchDescription()
#     ld.add_action(gazebo)
#     ld.add_action(controller_node)
#     return ld



#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('line_following')
    # Use the custom world file from the package's "worlds" folder (named "world.world")
    world_file = os.path.join(pkg_share, 'worlds', 'world.world')
    
    # Log the world file path for debugging purposes
    log_world = LogInfo(msg="Using world file: " + world_file)
    
    # Launch Gazebo with the custom world file and the ROS factory plugin for ROS integration
    gazebo_process = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Launch the line following controller node
    line_following_node = Node(
        package='line_following',
        executable='controller',
        name='line_following',
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(log_world)
    ld.add_action(gazebo_process)
    ld.add_action(line_following_node)
    return ld

if __name__ == '__main__':
    generate_launch_description()

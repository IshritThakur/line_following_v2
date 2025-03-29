# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import ExecuteProcess
# from launch_ros.actions import Node
# 
# def generate_launch_description():
#     # Get the package share directory (assumes your world file is installed there)
#     pkg_share = get_package_share_directory('line_following')
#     # Use your custom world file from the installed share directory
#     world_file = os.path.join(pkg_share, 'world', 'world.xml')
#     
#     # Launch Gazebo with the custom world file
#     gazebo = ExecuteProcess(
#         cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
#         output='screen'
#     )
#     
#     # Launch the controller node (make sure your entry point is correct)
#     controller_node = Node(
#         package='line_following',
#         executable='controller',
#         output='screen'
#     )
#     
#     ld = LaunchDescription()
#     ld.add_action(gazebo)
#     ld.add_action(controller_node)
#     return ld

#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('line_following')
    # Use the custom world file from the package's "worlds" folder (named "world.world")
    world_file = os.path.join(pkg_share, 'worlds', 'world.world')
    
    # Log the world file path for debugging purposes
    log_world = LogInfo(msg="Using world file: " + world_file)
    
    # Set the GAZEBO_MODEL_PATH environment variable so Gazebo can find your models
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(pkg_share, 'models')
    )
    
    # Launch Gazebo with the custom world file and the ROS factory plugin
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
    
    # Automatically call the start_line_follower service after 5 seconds
    start_service_call = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'service', 'call', '/start_line_follower', 'std_srvs/srv/Empty', '{}'],
                output='screen'
            )
        ]
    )

    ld = LaunchDescription()
    ld.add_action(log_world)
    ld.add_action(set_gazebo_model_path)
    ld.add_action(gazebo_process)
    ld.add_action(line_following_node)
    ld.add_action(start_service_call)
    return ld

if __name__ == '__main__':
    generate_launch_description()

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
    world_file = os.path.join(pkg_share, 'worlds', 'world.world')
    
    # Log world file path for debugging.
    log_world = LogInfo(msg="Using world file: " + world_file)
    
    # Set GAZEBO_MODEL_PATH so Gazebo can find your models.
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(pkg_share, 'models')
    )
    
    # Launch Gazebo with your world file and factory plugin.
    gazebo_process = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    
    # Spawn first robot ("robot") at (0, 0, 0.01)
    spawn_robot1 = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-entity', 'robot1',
            '-robot_namespace', 'robot1',
            '-file', os.path.join(pkg_share, 'models', 'robot.sdf'),
            '-x', '0', '-y', '0', '-z', '0.01'
         ],
        output='screen'
    )

    # Spawn the second robot using robot2.sdf (for "robot2")
    spawn_robot2 = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-entity', 'robot2',
            '-robot_namespace', 'robot2',
            '-file', os.path.join(pkg_share, 'models', 'robot2.sdf'),
            '-x', '1', '-y', '0', '-z', '0.01'
        ],
        output='screen'
    )
    
    # Launch controller node for robot1.
    # Note: The controller subscribes to "camera/image_raw" (relative topic),
    # so in the namespace "robot" it will resolve to "/robot/camera/image_raw".
    controller_robot1 = Node(
        package='line_following',
        executable='controller',
        name='controller',
        namespace='robot',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # Launch controller node for robot2.
    controller_robot2 = Node(
        package='line_following',
        executable='controller',
        name='controller',
        namespace='robot2',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # Call the start_line_follower service for each robot after a 10-second delay.
    start_service_robot1 = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'service', 'call', '/robot/start_line_follower', 'std_srvs/srv/Empty', '{}'],
                output='screen'
            )
        ]
    )
    
    start_service_robot2 = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'service', 'call', '/robot2/start_line_follower', 'std_srvs/srv/Empty', '{}'],
                output='screen'
            )
        ]
    )
    
    ld = LaunchDescription()
    ld.add_action(log_world)
    ld.add_action(set_gazebo_model_path)
    ld.add_action(gazebo_process)
    ld.add_action(spawn_robot1)
    ld.add_action(spawn_robot2)
    ld.add_action(controller_robot1)
    ld.add_action(controller_robot2)
    ld.add_action(start_service_robot1)
    ld.add_action(start_service_robot2)
    
    return ld

if __name__ == '__main__':
    generate_launch_description()

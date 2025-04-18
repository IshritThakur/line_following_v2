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



#REVERT TO AFTER SS
# import os
# import yaml
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import ExecuteProcess, LogInfo, SetEnvironmentVariable, TimerAction, GroupAction
# from launch_ros.actions import Node, PushRosNamespace

# def generate_launch_description():
#     pkg_share = get_package_share_directory('line_following')
#     world_file = os.path.join(pkg_share, 'worlds', 'world.world')
#     log_world = LogInfo(msg="Using world file: " + world_file)
#     set_gazebo_model_path = SetEnvironmentVariable(
#         name='GAZEBO_MODEL_PATH',
#         value=os.path.join(pkg_share, 'models')
#     )
#     gazebo_process = ExecuteProcess(
#         cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
#         output='screen'
#     )
#     yaml_file = os.path.join(pkg_share, 'config', 'initial_poses.yaml')
#     with open(yaml_file, 'r') as f:
#         poses = yaml.safe_load(f)
#     robot_configs = poses[2]

#     group_actions = []
#     start_service_actions = []

#     for robot_name, pose in robot_configs.items():
#         sdf_filename = f"{robot_name}.sdf"
#         sdf_path = os.path.join(pkg_share, 'models', sdf_filename)
#         print(f"Spawning {robot_name} from file {sdf_path} with pose {pose}")

#         spawn_robot = ExecuteProcess(
#             cmd=[
#                 'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
#                 '-entity', robot_name,
#                 '-robot_namespace', robot_name,
#                 '-file', sdf_path,
#                 '-x', str(pose["x"]),
#                 '-y', str(pose["y"]),
#                 '-z', '0.01',
#                 '-Y', str(pose["yaw"])
#             ],
#             output='screen'
#         )

#         controller_node = Node(
#             package='line_following',
#             executable='controller',
#             name='controller',
#             namespace=robot_name,
#             output='screen',
#             parameters=[{'use_sim_time': True}]
#         )

#         group = GroupAction([
#             PushRosNamespace(robot_name),
#             spawn_robot,
#             controller_node
#         ])
#         group_actions.append(group)

#         start_service = TimerAction(
#             period=10.0,
#             actions=[
#                 ExecuteProcess(
#                     cmd=[
#                         'ros2', 'service', 'call',
#                         f'/{robot_name}/start_line_follower', 'std_srvs/srv/Empty', '{}'
#                     ],
#                     output='screen'
#                 )
#             ]
#         )
#         start_service_actions.append(start_service)

#     ld = LaunchDescription()
#     ld.add_action(log_world)
#     ld.add_action(set_gazebo_model_path)
#     ld.add_action(gazebo_process)
#     for group in group_actions:
#         ld.add_action(group)
#     for ts in start_service_actions:
#         ld.add_action(ts)

#     return ld

# if __name__ == '__main__':
#     generate_launch_description()



import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('line_following')

    world_file = os.path.join(pkg_share, 'worlds', 'world.world')

    log_world = LogInfo(msg="Using world file: " + world_file)


    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(pkg_share, 'models')
    )

    # Launch Gazebo with the specified world and the ROS factory plugin
    gazebo_process = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Spawn a single robot using the spawn_entity node
    spawn_robot = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-entity', 'line_following_robot',
            '-robot_namespace', 'line_following_robot',
            '-file', os.path.join(pkg_share, 'models', 'robot.sdf'),
            '-x', '0', '-y', '0', '-z', '0.01'
        ],
        output='screen'
    )

    # Launch the line-following controller node
    line_following_node = Node(
        package='line_following',
        executable='controller',
        name='line_following',
        namespace='line_following_robot',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Optionally, start the line follower service after 10 seconds
    start_service_call = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'service', 'call', '/line_following_robot/start_line_follower', 'std_srvs/srv/Empty', '{}'],
                output='screen'
            )
        ]
    )

    ld = LaunchDescription()
    ld.add_action(log_world)
    ld.add_action(set_gazebo_model_path)
    ld.add_action(gazebo_process)
    ld.add_action(spawn_robot)
    ld.add_action(line_following_node)
    ld.add_action(start_service_call)






    return ld

if __name__ == '__main__':
    generate_launch_description()
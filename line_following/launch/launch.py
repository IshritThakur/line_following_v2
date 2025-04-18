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



from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('line_following')
    world_file = os.path.join(pkg_share, 'worlds', 'world.world')
    log_world = LogInfo(msg="Using world file: " + world_file)

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(pkg_share, 'models')
    )

    gazebo_process = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

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

    spawn_robot2 = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-entity', 'robot2',
            '-robot_namespace', 'robot2',
            '-file', os.path.join(pkg_share, 'models', 'robot.sdf'),
            '-x', '4', '-y', '0', '-z', '0.01'
        ],
        output='screen'
    )

    urdf_file = os.path.join(pkg_share, 'models', 'robot.urdf')
    if os.path.exists(urdf_file):
        with open(urdf_file, 'r') as infp:
            robot_description = infp.read()
    else:
        robot_description = ""

    rsp_robot1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='robot1',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    rsp_robot2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='robot2',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    line_following_node_robot1 = Node(
        package='line_following',
        executable='controller',
        name='line_following',
        namespace='robot1',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    line_following_node_robot2 = Node(
        package='line_following',
        executable='controller',
        name='line_following',
        namespace='robot2',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    start_service_call_robot1 = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'service', 'call', '/robot1/start_line_follower', 'std_srvs/srv/Empty', '{}'],
                output='screen'
            )
        ]
    )

    start_service_call_robot2 = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'service', 'call', '/robot2/start_line_follower', 'std_srvs/srv/Empty', '{}'],
                output='screen'
            )
        ]
    )

    # Now create and populate the LaunchDescription
    ld = LaunchDescription()
    ld.add_action(log_world)
    ld.add_action(set_gazebo_model_path)
    ld.add_action(gazebo_process)
    ld.add_action(spawn_robot1)
    ld.add_action(spawn_robot2)
    ld.add_action(rsp_robot1)
    ld.add_action(rsp_robot2)
    ld.add_action(line_following_node_robot1)
    ld.add_action(line_following_node_robot2)
    ld.add_action(start_service_call_robot1)
    ld.add_action(start_service_call_robot2)

    return ld

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
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, SetEnvironmentVariable, TimerAction, GroupAction
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    pkg_share = get_package_share_directory('line_following')
    # World file
    world_file = os.path.join(pkg_share, 'worlds', 'world.world')
    log_world = LogInfo(msg="Using world file: " + world_file)

    # Set Gazebo model path
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(pkg_share, 'models')
    )

    # Launch Gazebo with world file and factory plugin
    gazebo_process = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Load initial poses from YAML file
    yaml_file = os.path.join(pkg_share, 'config', 'initial_poses.yaml')
    with open(yaml_file, 'r') as f:
        poses = yaml.safe_load(f)
    # Use configuration "2"
    robot_configs = poses["2"]

    group_actions = []
    start_service_actions = []

    for robot_name, pose in robot_configs.items():
        # Decide which SDF file to use based on the robot name.
        sdf_filename = 'robot.sdf'
        if robot_name.lower() == 'robot2':
            sdf_filename = 'robot2.sdf'
        sdf_path = os.path.join(pkg_share, 'models', sdf_filename)

        # Create spawn command with parameters from YAML.
        spawn_robot = ExecuteProcess(
            cmd=[
                'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                '-entity', robot_name,
                '-robot_namespace', robot_name,
                '-file', sdf_path,
                '-x', str(pose["x"]),
                '-y', str(pose["y"]),
                '-z', '0.01',
                '-Y', str(pose["yaw"])
            ],
            output='screen'
        )

        # Create controller node (ensure your controller subscribes to relative topics)
        controller_node = Node(
            package='line_following',
            executable='controller',
            name='controller',
            namespace=robot_name,
            output='screen',
            parameters=[{'use_sim_time': True}]
        )

        # Combine spawn and controller node under the robot namespace
        group = GroupAction([
            PushRosNamespace(robot_name),
            spawn_robot,
            controller_node
        ])
        group_actions.append(group)

        # Create a timer action to call the start_line_follower service for this robot.
        start_service = TimerAction(
            period=10.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'service', 'call',
                        f'/{robot_name}/start_line_follower', 'std_srvs/srv/Empty', '{}'
                    ],
                    output='screen'
                )
            ]
        )
        start_service_actions.append(start_service)

    ld = LaunchDescription()
    ld.add_action(log_world)
    ld.add_action(set_gazebo_model_path)
    ld.add_action(gazebo_process)
    for group in group_actions:
        ld.add_action(group)
    for ts in start_service_actions:
        ld.add_action(ts)

    return ld

if __name__ == '__main__':
    generate_launch_description()

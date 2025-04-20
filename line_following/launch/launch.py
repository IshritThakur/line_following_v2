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

    ld.add_action(log_world)
    ld.add_action(set_gazebo_model_path)
    ld.add_action(gazebo_process)
    ld.add_action(spawn_robot)
    ld.add_action(line_following_node)
    ld.add_action(start_service_call)

    return ld

if __name__ == '__main__':
    generate_launch_description()
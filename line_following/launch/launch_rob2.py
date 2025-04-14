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
    # NOTE: Gazebo doesn't need to be launched again if it's already running.
    spawn_robot2 = ExecuteProcess(
        cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
             '-entity', 'robot2',
             '-robot_namespace', 'robot2',
             '-file', os.path.join(pkg_share, 'models', 'robot2.sdf'),
             '-x', '1', '-y', '0', '-z', '0.01'],
        output='screen'
    )
    controller_robot2 = Node(
        package='line_following',
        executable='controller',
        name='controller',
        namespace='robot2',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    start_service_robot2 = TimerAction(
        period=10.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/robot2/start_line_follower', 'std_srvs/srv/Empty', '{}'],
            output='screen'
        )]
    )
    ld = LaunchDescription()
    ld.add_action(log_world)
    ld.add_action(set_gazebo_model_path)
    ld.add_action(spawn_robot2)
    ld.add_action(controller_robot2)
    ld.add_action(start_service_robot2)
    return ld

if __name__ == '__main__':
    generate_launch_description()

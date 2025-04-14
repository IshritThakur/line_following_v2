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
    gazebo_process = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    spawn_robot1 = ExecuteProcess(
        cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
             '-entity', 'robot1',
             '-robot_namespace', 'robot1',
             '-file', os.path.join(pkg_share, 'models', 'robot.sdf'),
             '-x', '0', '-y', '0', '-z', '0.01'],
        output='screen'
    )
    controller_robot1 = Node(
        package='line_following',
        executable='controller',
        name='controller',
        namespace='robot1',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    start_service_robot1 = TimerAction(
        period=10.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/robot1/start_line_follower', 'std_srvs/srv/Empty', '{}'],
            output='screen'
        )]
    )
    ld = LaunchDescription()
    ld.add_action(log_world)
    ld.add_action(set_gazebo_model_path)
    ld.add_action(gazebo_process)
    ld.add_action(spawn_robot1)
    ld.add_action(controller_robot1)
    ld.add_action(start_service_robot1)
    return ld

if __name__ == '__main__':
    generate_launch_description()

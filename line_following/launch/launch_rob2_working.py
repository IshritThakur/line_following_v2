#!/usr/bin/env python3
"""
Launch Gazebo, spawn exactly two line-following robots (robot1, robot2),
and bring up one controller node per robot.  Ten seconds after start-up the
launch file calls /robot?/start_line_follower so both robots begin moving.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node


# ── helpers ────────────────────────────────────────────────────────────────
def spawn(ns: str, pkg_share: str, x: float, y: float):
    """Return an ExecuteProcess that spawns one robot model."""
    return ExecuteProcess(
        cmd=[
            'bash', '-lc',
            (
                'ros2 run gazebo_ros spawn_entity.py '
                f'-entity {ns} '
                f'-robot_namespace {ns} '
                f'-file {os.path.join(pkg_share, "models", "line_following_robot", "robot.sdf")} '
                f'-x {x} -y {y} -z 0.01 || true'
            )
        ],
        output='screen',
    )


def controller(ns: str):
    """Return one line-follower node living in *ns*."""
    return Node(
        package='line_following',
        executable='controller',
        name='line_follower',
        namespace=ns,
        output='screen',
        parameters=[{'use_sim_time': True}],
        # NO remappings: inside /robot? the default camera topic is already
        # /robot?/camera/image_raw, which is exactly what we want.
    )


def auto_start(ns: str, delay: float = 10.0):
    """Call /robot?/start_line_follower after *delay* seconds."""
    return TimerAction(
        period=delay,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'service', 'call',
                    f'/{ns}/start_line_follower',
                    'std_srvs/srv/Empty', '{}'
                ],
                output='screen',
            )
        ],
    )


# ── launch description ────────────────────────────────────────────────────
def generate_launch_description():
    pkg_share  = get_package_share_directory('line_following')
    world_file = os.path.join(pkg_share, 'worlds', 'world.world')

    ld = LaunchDescription()

    # Gazebo core -----------------------------------------------------------
    ld.add_action(LogInfo(msg=f'Using world: {world_file}'))
    ld.add_action(SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(pkg_share, 'models')
    ))
    ld.add_action(ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    ))

    # Two robots ------------------------------------------------------------
    robots = [
        {'ns': 'robot1', 'x': 0.0, 'y': 0.0},
        {'ns': 'robot2', 'x': 1.5, 'y': 0.0},
    ]

    for rb in robots:
        ld.add_action(spawn(rb['ns'], pkg_share, rb['x'], rb['y']))
        ld.add_action(controller(rb['ns']))
        ld.add_action(auto_start(rb['ns']))

    return ld


if __name__ == '__main__':
    generate_launch_description()

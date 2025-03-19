import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # Get the share directory of gazebo_ros package, which contains world files
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    # Use an empty world from gazebo_ros (you can choose another world if desired)
    world_file = os.path.join(gazebo_ros_share, 'worlds', 'empty.world')
    
    # Launch Gazebo with the specified world file.
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    
    # Optionally, if you want to launch your robot controller node in simulation,
    # uncomment the following lines and adjust the executable name as needed.
    #
    # controller_node = Node(
    #     package='line_follower',
    #     executable='controller',
    #     output='screen'
    # )
    
    ld = LaunchDescription()
    ld.add_action(gazebo)
    # ld.add_action(controller_node)  # Uncomment to also launch your controller node
    return ld

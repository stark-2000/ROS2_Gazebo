from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    record_flag = LaunchConfiguration('record_flag')
    return LaunchDescription([
        DeclareLaunchArgument(
            'record_flag',
            default_value='False'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch'),'/turtlebot3_world.launch.py'])
        ),
        Node(
            package='ros2_gazebo',
            executable='Walker_Algo',
        ),
        ExecuteProcess(
        condition=IfCondition(record_flag),
        cmd=[
            'ros2', 'bag', 'record', '-o Walker_Algo_bag', '-a', '-x "/camera.+"'
        ],
        shell=True
        )
    ])
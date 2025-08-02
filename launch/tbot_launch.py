#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明launch参数
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.8.110',
        description='TBot robot IP address'
    )
    
    # 创建TBot节点
    tbot_node = Node(
        package='tbot_sdk',
        executable='tbot_node',
        name='tbot_node',
        output='screen',
        parameters=[{
            'robot_ip': LaunchConfiguration('robot_ip')
        }],
        remappings=[
            ('odom', '/odom'),
            ('scan', '/scan'),
            ('cmd_vel', '/cmd_vel'),
            ('goal_pose', '/goal_pose'),
            ('status', '/tbot/status')
        ]
    )
    
    return LaunchDescription([
        robot_ip_arg,
        tbot_node
    ]) 
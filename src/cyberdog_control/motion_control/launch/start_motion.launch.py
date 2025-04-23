#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


# 一键启动仿真环境，控制节点，可视化，二维码服务

def generate_launch_description():

    # 启动仿真环境
    launch_sim_script = ExecuteProcess(
    cmd=['python3', '/home/cennum/cyberdog_sim/src/cyberdog_simulator/cyberdog_gazebo/script/launchsim.py'],
    output='screen'
    )

    # 启动motion_manager节点
    motion_manager_node = Node(
        package='motion_manager',
        executable='motion_manager',
        name='motion_manager',
        output='screen'
    )

    # 等待2秒后激活状态机
    activate_state_machine = ExecuteProcess(
        cmd=['ros2', 'service', 'call', 
             '/motion_managermachine_service',
             'protocol/srv/FsMachine',
             '{target_state: "Active"}'],
        output='screen'
    )

    # 创建状态机延时动作
    delayed_activation = TimerAction(
        period=2.0,  # 延时2秒
        actions=[activate_state_machine]
    )

    # 启动cyberdog_walk节点
    cyberdog_walk_node = Node(
        package='motion_control',
        executable='cyberdog_walk',
        name='cyberdog_walk_server',
        output='screen'
    )

    # 启动二维码服务端节点
    cyberdog_qrcode_server_node = Node(
        package='lane_detection',
        executable='qrscan',
        name='qrcode_server',
        output='screen'
    )

    # 发布静态tf
    map_to_odom_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_broadcaster',
        arguments=['0', '0', '0', '1.57079632679', '0', '0', 'map', 'odom']
    )


    return LaunchDescription([
        launch_sim_script,
        motion_manager_node,
        delayed_activation,
        cyberdog_walk_node,
        map_to_odom_static_tf,

    ])
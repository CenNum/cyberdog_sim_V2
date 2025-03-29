#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

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

    # 创建延时动作
    delayed_activation = TimerAction(
        period=2.0,  # 延时2秒
        actions=[activate_state_machine]
    )

    # 启动cyberdog_walk节点
    cyberdog_walk_node = Node(
        package='motion_control',
        executable='cyberdog_walk',
        name='cyberdog_walk',
        output='screen'
    )

    # 创建启动cyberdog_walk节点的延时动作
    delayed_cyberdog_walk = TimerAction(
        period=10.0,  # 延时8秒
        actions=[cyberdog_walk_node]
    )
    return LaunchDescription([
        launch_sim_script,
        motion_manager_node,
        delayed_activation,
        #delayed_cyberdog_walk
    ])
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

    # 创建状态机延时动作
    delayed_activation = TimerAction(
        period=2.0,  # 延时2秒
        actions=[activate_state_machine]
    )

    # 启动cyberdog_walk节点
    custom_walk_node = Node(
        package='motion_control',
        executable='custom_walk',
        name='custom_walk',
        output='screen'
    )


    # 发送控制命令1
    send_control_msg1 = ExecuteProcess(
        cmd=['ros2', 'param', 'set',
            '/custom_walk',  # 注意这里分成独立参数
            'motion_sequence',
            r'''[
                '["stop", 1.0, [0.0, 0.0, 0.0]]'
            ]'''.replace('\n', '').replace(' ', ''),],
        output='screen'
    )

    # 发送控制命令2
    send_control_msg2 = ExecuteProcess(
        cmd=['ros2', 'param', 'set',
            '/custom_walk',  # 注意这里分成独立参数
            'motion_sequence',
            r'''[
                '["forward", 3.0, [0.6, 0.0, 0.0]]',
                '["strafe", 1.5, [0.0, 0.4, 0.0]]',
                '["rotate", 3.14, [0.0, 0.0, 0.5]]',
                '["backward", 4.0, [-0.6, 0.0, 0.0]]',
                '["stop", 1.0, [0.0, 0.0, 0.0]]'
            ]'''.replace('\n', '').replace(' ', ''),],
        output='screen'
    )
    # 创建启动custom_walk节点的延时动作
    delayed_send_control_msg1 = TimerAction(
        period=10.0,  # 延时2秒
        actions=[send_control_msg1]
    )

    # 创建启动custom_walk节点的延时动作
    delayed_send_control_msg2 = TimerAction(
        period=18.0,  # 延时8秒
        actions=[send_control_msg2]
    )
    

    return LaunchDescription([
        launch_sim_script,
        motion_manager_node,
        delayed_activation,
        #custom_walk_node,
        #delayed_send_control_msg1,
        #delayed_send_control_msg2
    ])
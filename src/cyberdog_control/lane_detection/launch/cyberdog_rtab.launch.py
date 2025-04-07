from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share_dir = get_package_share_directory('lane_detection')
    database_path = os.path.join(pkg_share_dir, 'database', 'rtabmap.db')
    map_save_dir = os.path.join(pkg_share_dir, 'maps')
    
    # 创建必要的目录
    os.makedirs(os.path.dirname(database_path), exist_ok=True)
    os.makedirs(map_save_dir, exist_ok=True)
    
    rtabmap_node = Node(
        package='rtabmap_ros',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            # 您原有的所有参数保持不变...
            'frame_id': 'D435_camera_link',
            'odom_frame_id': 'vodom',
            'map_frame_id': 'map',
            'database_path': database_path,
    
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_scan': False,
            'subscribe_imu': True,
            'subscribe_odom': True,
    
            # RGBD设置
            'Reg/Strategy': '0',
            'Reg/Force3DoF': 'False',
            'RGBD/ProximityBySpace': 'True',
            'RGBD/AngularUpdate': '0.1',
            'RGBD/LinearUpdate': '0.1',
            'RGBD/OptimizeFromGraphEnd': 'True',
    
            # 视觉里程计设置
            'Vis/EstimationType': '0',
            'Vis/MinInliers': '20',
            'Vis/MaxDepth': '10.0',
            'Vis/MaxFeatures': '1000',
    
            # 地图设置
            'Grid/FromDepth': 'True',
            'Grid/2D': 'True',
            'Grid/CellSize': '0.05',
    
            # 回环检测设置
            'Loop/VerifyLoopClosure': True,
            'Loop/BundleAdjustment': 1,
    
            # 内存设置
            'Mem/IncrementalMemory': 'True',
    
            # 其他设置
            'Rtabmap/DetectionRate': '1.0',
            'RGBD/CreateOccupancyGrid': 'True',
    
            # QoS设置
            'qos_image': 2,
            'qos_imu': 2,
            'qos_odom': 2,
    
            # TF设置
            'publish_tf': True,
            'publish_map_odom': True,
            'tf_delay': 0.05,
            'wait_for_transform': 0.2
        }],
        remappings=[
            ('rgb/image', '/D435_camera/image_raw'),
            ('rgb/camera_info', '/D435_camera/camera_info'),
            ('depth/image', '/D435_camera/depth/image_raw'),
            ('depth/camera_info', '/D435_camera/depth/camera_info'),
            ('imu', '/imu'),
            ('grid_map', 'map')
        ]
    )
    
    rtabmapviz_node = Node(
        package='rtabmap_ros',
        executable='rtabmapviz',
        name='rtabmapviz',
        output='screen',
        parameters=[{
            'frame_id': 'D435_camera_link',
            'odom_frame_id': 'vodom',
            'map_frame_id': 'map',
    
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_odom': True,
            'subscribe_imu': True,
    
            'qos_image': 2,
            'qos_imu': 2,
            'qos_odom': 2,
    
            'wait_for_transform': 0.2
        }],
        remappings=[
            ('rgb/image', '/D435_camera/image_raw'),
            ('rgb/camera_info', '/D435_camera/camera_info'),
            ('depth/image', '/D435_camera/depth/image_raw'),
            ('depth/camera_info', '/D435_camera/depth/camera_info'),
            ('imu', '/imu')
        ]
    )
    
    # 地图保存进程
    save_map_process = ExecuteProcess(
        cmd=['bash', '-c', f'''
            sleep 2
            echo "Saving map to {map_save_dir}/map"
            ros2 run nav2_map_server map_saver_cli -f {map_save_dir}/map --ros-args -p save_map_timeout:=10.0
            if [ $? -eq 0 ]; then
                echo "Map saved successfully to {map_save_dir}/map.pgm and {map_save_dir}/map.yaml"
                cp "{database_path}" "{map_save_dir}/rtabmap.db" 2>/dev/null || true
            else
                echo "Failed to save map"
            fi
        '''],
        output='screen'
    )
    
    # 注册退出事件处理器
    exit_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=rtabmap_node,
            on_exit=[save_map_process]
        )
    )
    
    # 延迟启动可视化节点
    delay_rtabmapviz = TimerAction(
        period=5.0,
        actions=[rtabmapviz_node]
    )
    
    # 静态TF发布
    map_to_vodom_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_vodom_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'vodom']
    )
    
    return LaunchDescription([
        rtabmap_node,
        delay_rtabmapviz,
        map_to_vodom_static_tf,
        exit_event_handler
    ])
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import struct

class PointCloudColorParser(Node):
    def __init__(self):
        super().__init__('d435_color_parser')
        self.processed = False
        self.shutdown_flag = False
        self.subscription = self.create_subscription(
            PointCloud2,
            '/D435_camera/points',
            self.pointcloud_callback,
            10  # QoS队列深度
        )
        self.get_logger().info("已订阅/D435_camera/points，等待数据...")

    def pointcloud_callback(self, msg):
        """处理第一个接收到的点云消息"""
        if self.processed:
            return
            
        self.processed = True
        self.get_logger().info("开始处理首帧点云数据...")
        
        try:
            # 查找rgb字段
            rgb_field = next((f for f in msg.fields if f.name == "rgb"), None)
            if not rgb_field:
                self.get_logger().error("点云数据中没有找到rgb字段!")
                return

            # 解析参数
            point_step = msg.point_step
            offset_rgb = rgb_field.offset
            total_points = msg.width * msg.height
            data = bytes(msg.data)
            parse_as_uint32 = (rgb_field.datatype == 6)

            # 存储所有RGB值
            rgb_values = []
            
            for i in range(total_points):
                try:
                    # 提取点数据
                    start_idx = i * point_step
                    end_idx = start_idx + point_step
                    point_data = data[start_idx:end_idx]
                    
                    # 提取RGB字节
                    rgb_bytes = point_data[offset_rgb : offset_rgb + 4]

                    # 解析RGB值
                    if parse_as_uint32:
                        rgb_packed = struct.unpack('<I', rgb_bytes)[0]
                        r = (rgb_packed >> 16) & 0xFF
                        g = (rgb_packed >> 8) & 0xFF
                        b = rgb_packed & 0xFF
                    else:
                        r = rgb_bytes[0]
                        g = rgb_bytes[1]
                        b = rgb_bytes[2]
                    
                    rgb_values.append((r, g, b))
                    
                except Exception as e:
                    self.get_logger().warning(f"解析点[{i}]时出错: {str(e)}")
                    continue

            # 保存到文件
            with open('rgb_values.csv', 'w') as f:
                f.write("R,G,B\n")
                for r, g, b in rgb_values:
                    f.write(f"{r},{g},{b}\n")
            
            self.get_logger().info(
                f"成功保存{len(rgb_values)}/{total_points}个RGB值到当前目录的rgb_values.csv文件"
            )

        except Exception as e:
            self.get_logger().error(f"处理失败: {str(e)}")
        finally:
            self.shutdown_flag = True  # 触发关闭

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudColorParser()
    
    try:
        # 使用spin_once实现可中断循环
        while rclpy.ok() and not node.shutdown_flag:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("节点已关闭")

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import ctypes
from ctypes import c_char_p, c_float, c_double, c_int, POINTER, Structure

class ORBSLAM3Node(Node):
    def __init__(self):
        super().__init__('orb_slam3_node')
        
        # 创建窗口显示图像
        cv2.namedWindow('RGB Image', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Depth Image', cv2.WINDOW_NORMAL)
        
        # 加载ORB-SLAM3库
        wrapper_path = '/home/cennum/cyberdog_sim/src/ORB_SLAM3/lib/liborb_slam3_wrapper.so'
        self.lib = ctypes.CDLL(wrapper_path)
        
        # 设置QoS配置
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 订阅D435相机话题
        self.rgb_sub = self.create_subscription(
            Image,
            '/D435_camera/image_raw',  # 修改为D435的RGB话题
            self.rgb_callback,
            qos_profile)

        self.depth_sub = self.create_subscription(
            Image,
            '/D435_camera/depth/image_raw',  # 修改为D435的深度话题
            self.depth_callback,
            qos_profile)
            
        self.bridge = CvBridge()
        
        # 初始化ORB-SLAM3
        vocab_path = "/home/cennum/cyberdog_sim/src/ORB_SLAM3/Vocabulary/ORBvoc.txt"
        config_path = "/home/cennum/cyberdog_sim/src/ORB_SLAM3/Examples/RGB-D/RealSense_D435i.yaml"
        
        # 定义函数原型
        self.lib.createSystem.argtypes = [c_char_p, c_char_p, c_char_p]
        self.lib.createSystem.restype = ctypes.c_void_p
        
        self.lib.trackRGBD.argtypes = [
            ctypes.c_void_p,  # SLAM system pointer
            np.ctypeslib.ndpointer(dtype=np.uint8),  # RGB image
            np.ctypeslib.ndpointer(dtype=np.float32),  # Depth image
            c_double  # Timestamp
        ]
        
        # 创建SLAM系统
        self.slam = self.lib.createSystem(
            vocab_path.encode(),
            config_path.encode(),
            b"RGBD"
        )
        
        self.latest_rgb = None
        self.latest_depth = None
        self.frame_id = 0
        self.width = 640
        self.height = 480
        
    def rgb_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv_image = cv2.resize(cv_image, (self.width, self.height))  # 统一尺寸
            cv_image = np.ascontiguousarray(cv_image, dtype=np.uint8)  # 确保数据连续
            self.latest_rgb = cv_image
            cv2.imshow('RGB Image', cv_image)
            cv2.waitKey(1)
            self.process_frames()
        except Exception as e:
            self.get_logger().error(f'Error processing RGB image: {str(e)}')

            
    def depth_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")  # 确保是 32-bit float 格式
            depth_image = cv2.resize(depth_image, (self.width, self.height))  # 统一尺寸

            # **移除 NaN/Inf 数据**
            depth_image = np.nan_to_num(depth_image, nan=0.0, posinf=10.0, neginf=0.0)

            # **防止超大深度值**
            depth_image = np.clip(depth_image, 0.1, 10.0)  # 限制深度在 0.1m ~ 10m 内

            # **确保数据连续**
            self.latest_depth = np.ascontiguousarray(depth_image, dtype=np.float32)

            # 生成深度图显示
            depth_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
            depth_colormap = cv2.applyColorMap(depth_normalized.astype(np.uint8), cv2.COLORMAP_JET)

            cv2.imshow('Depth Image', depth_colormap)
            cv2.waitKey(1)

            self.process_frames()
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {str(e)}')



            
    def process_frames(self):
        if self.latest_rgb is not None and self.latest_depth is not None:
            try:
                # 确保数据格式正确
                rgb_data = np.ascontiguousarray(self.latest_rgb, dtype=np.uint8)
                depth_data = np.ascontiguousarray(self.latest_depth, dtype=np.float32)  # **保持 float32**

                # **检查 depth_data 是否符合要求**
                print(f"Depth Image Shape: {depth_data.shape}, Type: {depth_data.dtype}, Min: {np.min(depth_data)}, Max: {np.max(depth_data)}")

                if rgb_data.shape != (self.height, self.width, 3) or depth_data.shape != (self.height, self.width):
                    self.get_logger().error(f"Image size mismatch! RGB: {rgb_data.shape}, Depth: {depth_data.shape}")
                    return

                # **确保 depth_data 没有 NaN 或 Inf**
                depth_data = np.nan_to_num(depth_data, nan=0.0, posinf=10.0, neginf=0.0)

                # 传递数据给 ORB-SLAM3
                timestamp = self.frame_id * 0.033  # 假设 30 FPS
                self.frame_id += 1

                self.lib.trackRGBD(self.slam, rgb_data, depth_data, timestamp)

                # 清空数据，避免内存泄漏
                self.latest_rgb = None
                self.latest_depth = None
            except Exception as e:
                self.get_logger().error(f'Error in SLAM processing: {str(e)}')




            
    def __del__(self):
        if hasattr(self, 'slam') and self.slam:
            self.lib.System_Shutdown.argtypes = [ctypes.c_void_p]
            self.lib.System_Shutdown(self.slam)  # 释放 SLAM 实例
            self.slam = None  # 避免野指针
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = ORBSLAM3Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from msgs_lane.srv import Qrcode
import cv2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np
import ament_index_python
import os

class QRScannerService(Node):
    def __init__(self):
        super().__init__('qr_scanner_service')
        

        # 初始化服务和订阅
        self.srv = self.create_service(
            Qrcode, 
            'Qrcode', 
            self.scan_qr_callback
        )
        
        self.bridge = CvBridge()
        self.latest_image = None
    
        # 设置QoS
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 订阅相机图像
        self.image_sub = self.create_subscription(
            Image,
            '/D435_camera/image_raw',
            self.image_callback,
            qos_profile
        )
    
        # 初始化WeChatQRCode检测器
        package_path = ament_index_python.get_package_share_directory('lane_detection')
        model_dir = os.path.join(package_path, 'models')
        
        detector_path = os.path.join(model_dir, "detect.prototxt")
        detector_model = os.path.join(model_dir, "detect.caffemodel")
        super_resolution_path = os.path.join(model_dir, "sr.prototxt")
        super_resolution_model = os.path.join(model_dir, "sr.caffemodel")
        
        self.get_logger().info(f'模型路径: {model_dir}')
        
        try:
            self.detector = cv2.wechat_qrcode_WeChatQRCode(
                detector_path,
                detector_model,
                super_resolution_path,
                super_resolution_model
            )
        except Exception as e:
            self.get_logger().error(f'初始化WeChatQRCode失败: {str(e)}')
            raise e
        
        self.get_logger().info('QR扫描服务已启动')
        
    def image_callback(self, msg):
        self.latest_image = msg
    
    def extract_and_resize_qr(self, image):
        """使用WeChatQRCode检测二维码并调整大小到640x480"""
        try:
            # 使用WeChatQRCode检测
            decoded_texts, points = self.detector.detectAndDecode(image)
            
            if not decoded_texts or not points or len(points[0]) < 4:
                return None, None, None
                
            # 获取第一个检测到的二维码的边界点
            qr_points = points[0]
            
            # 计算边界框
            x_coords = qr_points[:, 0]
            y_coords = qr_points[:, 1]
            x = int(min(x_coords))
            y = int(min(y_coords))
            w = int(max(x_coords) - x)
            h = int(max(y_coords) - y)
            
            # 从图像中裁剪出二维码区域（添加边距）
            margin = 10
            x = max(0, x - margin)
            y = max(0, y - margin)
            w = min(image.shape[1] - x, w + 2*margin)
            h = min(image.shape[0] - y, h + 2*margin)
            
            qr_region = image[y:y+h, x:x+w]
            
            # 调整大小到640x480
            resized_qr = cv2.resize(qr_region, (640, 480), interpolation=cv2.INTER_CUBIC)
            
            return resized_qr, (x, y, w, h), decoded_texts[0]
            
        except Exception as e:
            self.get_logger().error(f'提取二维码区域错误: {str(e)}')
            return None, None, None
        
    def scan_qr_callback(self, request, response):
        """处理扫描请求"""
        if self.latest_image is None:
            response.success = False
            response.content = ''
            return response
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")
            
            # 保存原始图像
            cv2.imwrite('/tmp/original.jpg', cv_image)
            
            # 获取图像尺寸
            height, width = cv_image.shape[:2]
            
            # 只取图像的上部分
            top = 0
            bottom = height  
            left = width // 6     
            right = width  
            
            # 裁剪图像
            roi = cv_image[top:bottom, left:right]
            
            # 保存裁剪后的图像
            cv2.imwrite('/tmp/cropped.jpg', roi)
            
            # 放大裁剪区域
            roi_height, roi_width = roi.shape[:2]
            enlarged = cv2.resize(roi, (roi_width*4, roi_height*4), interpolation=cv2.INTER_CUBIC)
            
            # 保存放大后的图像
            cv2.imwrite('/tmp/enlarged.jpg', enlarged)
            
            # 提取二维码区域并调整大小
            resized_qr, coords, decoded_text = self.extract_and_resize_qr(enlarged)
            
            if resized_qr is not None and decoded_text:
                # 保存调整大小后的二维码区域
                cv2.imwrite('/tmp/resized_qr.jpg', resized_qr)
                
                response.success = True
                response.content = decoded_text
                self.get_logger().info(f'检测到QR码: {response.content}')
                
                # 在原始放大图像上标记检测结果
                if coords:
                    x, y, w, h = coords
                    cv2.rectangle(enlarged, (x, y), (x+w, y+h), (0,255,0), 3)
                    cv2.putText(enlarged, decoded_text, (x, y-10),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,255,0), 2)
                    cv2.imwrite('/tmp/detected_qr.jpg', enlarged)
            else:
                response.success = False
                response.content = ''
                self.get_logger().warn('未检测到QR码')
                    
        except Exception as e:
            self.get_logger().error(f'扫描错误: {str(e)}')
            response.success = False
            response.content = ''
            
        return response

def main():
    rclpy.init()
    service = QRScannerService()
    rclpy.spin(service)
    service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
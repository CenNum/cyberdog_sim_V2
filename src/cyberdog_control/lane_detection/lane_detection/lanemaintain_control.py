#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from protocol.msg import MotionServoCmd
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

class DogControl(Node):
    def __init__(self):
        super().__init__('dog_control')
        
        # 创建运动指令发布器
        self.publisher_ = self.create_publisher(
            MotionServoCmd, 
            'motion_servo_cmd', 
            10
        )
        
        # 配置QoS
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 创建图像订阅器
        self.image_sub = self.create_subscription(
            Image,
            '/D435_camera/image_raw',
            self.image_callback,
            qos_profile
        )
        
        self.bridge = CvBridge()
        
        # PID控制参数
        self.kp = 0.005
        self.ki = 0.0001
        self.kd = 0.001
        self.last_error = 0.0
        self.error_sum = 0.0
        
        # 运动参数
        self.forward_speed = 0.3  # 前进速度
        self.max_turn_speed = 0.2  # 最大转向速度
        self.line_center = None  # 黄线中心位置
        
        self.get_logger().info('DogControl node已启动')
    
    def detect_line(self, image):
        """检测黄线并返回中心位置"""
        try:
            height, width = image.shape[:2]
            
            # 转换到HSV颜色空间
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            
            # 黄线的HSV范围
            lower_yellow = np.array([20, 100, 100])
            upper_yellow = np.array([80, 255, 255])
            
            # 创建掩码
            mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
            
            # 形态学操作
            kernel = np.ones((5,5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # 找到轮廓
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                # 找到最大的轮廓
                largest_contour = max(contours, key=cv2.contourArea)
                
                # 计算轮廓的矩形
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    # 在图像上标记中心点
                    cv2.circle(image, (cx, cy), 5, (0, 255, 0), -1)
                    cv2.drawContours(image, [largest_contour], -1, (0, 255, 0), 2)
                    
                    return cx, mask
            
            return None, mask
            
        except Exception as e:
            self.get_logger().error(f'线检测错误: {str(e)}')
            return None, None
    
    def detect_nearest_area(self, image):
        """检测最近的可行驶区域并寻找无障碍路径"""
        try:
            height, width = image.shape[:2]
            
            # 只分析图像最下方区域
            roi_height = int(height * 0.3)
            roi_y = height - roi_height
            roi = image[roi_y:height, :]
            
            # 转换到HSV颜色空间并检测黄线
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            yellow_mask = cv2.inRange(hsv, 
                                    np.array([20, 100, 100]), 
                                    np.array([80, 255, 255]))
            
            # 形态学处理
            kernel = np.ones((5,5), np.uint8)
            yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)
            yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)
            
            # 获取可行驶区域
            walkable_mask = cv2.bitwise_not(yellow_mask)
            
            # 找到所有连通区域
            num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(walkable_mask)
            
            # 找到最近的有效区域
            valid_areas = []
            for i in range(1, num_labels):
                x, y, w, h, area = stats[i]
                if area < 1000:  # 过滤小区域
                    continue
                if y + h >= roi_height - 5:  # 确保区域接触底边
                    valid_areas.append({
                        'id': i,
                        'x': x,
                        'y': y,
                        'w': w,
                        'h': h,
                        'center': centroids[i],
                        'area': area
                    })
            
            if not valid_areas:
                return None, None, None, image
                
            result_image = image.copy()
            best_path = None
            min_cost = float('inf')
            
            # 机器人当前位置（图像底部中心）
            robot_x = width // 2
            robot_y = height
            
            # 为每个有效区域寻找最佳路径
            for area in valid_areas:
                center_x, center_y = area['center']
                center_x = int(center_x)
                center_y = int(center_y) + roi_y
                
                # 检查直线路径是否可行
                path_clear = True
                path_points = self.get_line_points(robot_x, robot_y, center_x, center_y)
                
                for px, py in path_points:
                    if py >= roi_y:  # 只检查ROI区域内的点
                        # 将全局坐标转换为ROI坐标
                        local_y = py - roi_y
                        if local_y >= 0 and local_y < roi_height:
                            if yellow_mask[local_y, px] > 0:
                                path_clear = False
                                break
                
                # 计算路径代价（考虑距离和转向角度）
                distance = np.sqrt((center_x - robot_x)**2 + (center_y - robot_y)**2)
                angle = abs(np.arctan2(center_y - robot_y, center_x - robot_x))
                path_cost = distance + angle * 50  # 角度权重可调整
                
                # 如果路径无障碍且代价更低，更新最佳路径
                if path_clear and path_cost < min_cost:
                    min_cost = path_cost
                    best_path = {
                        'area': area,
                        'center_x': center_x,
                        'center_y': center_y,
                        'distance': distance,
                        'points': path_points
                    }
            
            # 绘制结果
            if best_path is not None:
                area = best_path['area']
                area_mask = (labels == area['id']).astype(np.uint8) * 255
                
                # 显示可行驶区域
                result_image[roi_y:height, :][area_mask > 0] = \
                    cv2.addWeighted(
                        result_image[roi_y:height, :][area_mask > 0],
                        0.7,
                        np.full_like(
                            result_image[roi_y:height, :][area_mask > 0],
                            [0, 255, 0]
                        ),
                        0.3,
                        0
                    )
                
                # 绘制最佳路径
                for px, py in best_path['points']:
                    cv2.circle(result_image, (px, py), 1, (0, 255, 255), -1)
                
                # 绘制目标点
                cv2.circle(result_image, 
                        (best_path['center_x'], best_path['center_y']), 
                        5, (0, 0, 255), -1)
                
                # 显示距离信息
                cv2.putText(result_image, 
                        f'Distance: {int(best_path["distance"])}px',
                        (10, height-20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        2)
                
                return (best_path['center_x'], 
                    best_path['center_y'], 
                    best_path['distance'], 
                    result_image)
                
            return None, None, None, result_image
            
        except Exception as e:
            self.get_logger().error(f'区域检测错误: {str(e)}')
            return None, None, None, image
    
    def get_line_points(self, x1, y1, x2, y2):
        """获取两点之间的所有点（Bresenham算法）"""
        points = []
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        x, y = x1, y1
        sx = 1 if x2 > x1 else -1
        sy = 1 if y2 > y1 else -1
        
        if dx > dy:
            err = dx / 2.0
            while x != x2:
                points.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y2:
                points.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
                
        points.append((x2, y2))
        return points
    
    def calculate_control(self, center_x, center_y, distance, image_width):
        """根据区域中心计算控制命令"""
        try:
            if center_x is None or center_y is None:
                return 0.0, 0.0
            
            # 计算横向偏差
            center_error = (center_x - image_width/2) / (image_width/2)
            
            # PID控制转向
            p_term = self.kp * center_error
            self.error_sum += center_error
            i_term = self.ki * self.error_sum
            d_term = self.kd * (center_error - self.last_error)
            
            turn_speed = -(p_term + i_term + d_term)
            
            # 根据距离调整速度
            max_distance = 200  # 最大参考距离
            distance_factor = min(1.0, distance / max_distance)
            forward_speed = self.forward_speed * (1 - distance_factor * 0.5)
            
            # 限制速度范围
            turn_speed = np.clip(turn_speed, -self.max_turn_speed, self.max_turn_speed)
            forward_speed = np.clip(forward_speed, 0.2, self.forward_speed)
            
            self.last_error = center_error
            
            return turn_speed, forward_speed
            
        except Exception as e:
            self.get_logger().error(f'控制计算错误: {str(e)}')
            return 0.0, 0.0
    
    def image_callback(self, msg):
        """处理图像回调"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 检测最近的可行驶区域
            center_x, center_y, distance, result_image = self.detect_nearest_area(cv_image)
            
            if center_x is not None:
                # 计算控制命令
                turn_speed, forward_speed = self.calculate_control(
                    center_x, center_y, distance, cv_image.shape[1])
                
                # 创建并发送控制命令
                msg = self.create_motion_msg(forward_speed, 0.0, turn_speed)
                self.publisher_.publish(msg)
                
                self.get_logger().info(
                    f'中心点: ({center_x}, {center_y}), ' 
                    f'距离: {int(distance)}px, '
                    f'速度: {forward_speed:.2f}, '
                    f'转向: {turn_speed:.2f}'
                )
            else:
                # 没有检测到有效区域，停止
                msg = self.create_motion_msg(0.0, 0.0, 0.0)
                self.publisher_.publish(msg)
                self.get_logger().warn('未检测到有效区域，停止运动')
            
            # 显示结果
            cv2.imshow('Area Detection', result_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'图像处理错误: {str(e)}')
    
    def create_motion_msg(self, vel_x, vel_y, vel_yaw):
        """创建运动指令消息"""
        msg = MotionServoCmd()
        msg.motion_id = 308  # 中速行走
        msg.value = 2        # 垂直步态
        msg.cmd_type = 1
        msg.cmd_source = 2   # Vis源
        
        msg.vel_des = [vel_x, vel_y, vel_yaw]
        msg.rpy_des = [0.0] * 3
        msg.pos_des = [0.0] * 3
        msg.acc_des = [0.0] * 3
        msg.ctrl_point = [0.0] * 3
        msg.foot_pose = [0.0] * 3
        msg.step_height = [0.05, 0.05]
        
        return msg

def main(args=None):
    rclpy.init(args=args)
    node = DogControl()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('程序被用户中断')
    finally:
        # 发送停止指令
        stop_msg = MotionServoCmd()
        stop_msg.cmd_type = 2
        node.publisher_.publish(stop_msg)
        
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
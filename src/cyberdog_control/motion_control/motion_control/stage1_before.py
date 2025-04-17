#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from protocol.msg import MotionServoCmd
import math
import time
from msgs_lane.srv import Qrcode
from action_motion_client import MotionClient

class DogMotionControl(Node):
    def __init__(self):
        super().__init__('dog_motion_control')
        
        # 创建运动指令发布器
        self.motion_pub = self.create_publisher(
            MotionServoCmd, 
            'motion_servo_cmd', 
            10
        )
        
        self.get_logger().info('Dog motion control node started')
        
    def create_motion_msg(self, vel_x=0.0, vel_y=0.0, vel_yaw=0.0):
        """创建运动指令消息"""
        msg = MotionServoCmd()
        msg.motion_id = 308  # 中速行走
        msg.value = 2        # 垂直步态
        msg.cmd_type = 1     # 运动指令
        msg.cmd_source = 2   # 来源
        
        # 设置速度
        msg.vel_des = [vel_x, vel_y, vel_yaw]
        msg.rpy_des = [0.0] * 3
        msg.pos_des = [0.0] * 3
        msg.acc_des = [0.0] * 3
        msg.ctrl_point = [0.0] * 3
        msg.foot_pose = [0.0] * 3
        msg.step_height = [0.05, 0.05]
        
        return msg
    
    def stand_up(self):
        """执行站立动作"""
        try:
            msg = MotionServoCmd()
            msg.motion_id = 304  # 站立模式的motion_id
            msg.value = 2        # 垂直步态
            msg.cmd_type = 1     # 运动指令
            msg.cmd_source = 2   # 来源
            
            # 设置所有速度为0
            msg.vel_des = [0.0] * 3
            msg.rpy_des = [0.0] * 3
            msg.pos_des = [0.0] * 3
            msg.acc_des = [0.0] * 3
            msg.ctrl_point = [0.0] * 3
            msg.foot_pose = [0.0] * 3
            msg.step_height = [0.05, 0.05]
            
            # 发布站立命令
            self.get_logger().info('Executing stand up...')
            msg = self.create_motion_msg()
            self.motion_pub.publish(msg)
            time.sleep(2.3)  # 等待动作完成
            
            self.get_logger().info('Stand up completed')
            
        except Exception as e:
            self.get_logger().error(f'Error during stand up: {str(e)}')
        
    def move_forward(self, distance, speed=0.8):
        """向前移动指定距离"""
        start_time = self.get_clock().now()
        duration = distance / speed
        
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < duration:
            msg = self.create_motion_msg(vel_x=speed)
            self.motion_pub.publish(msg)
            time.sleep(0.1)  # 控制发布频率
            
        # 停止
        self.stop()
    
    def move_backward(self, distance, speed=0.5):
        """向后移动指定距离"""
        start_time = self.get_clock().now()
        duration = distance / speed
        
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < duration:
            msg = self.create_motion_msg(vel_x=-speed)  # 负值表示向后移动
            self.motion_pub.publish(msg)
            time.sleep(0.1)  # 控制发布频率
            
        # 停止
        self.stop()
    
    def move_left(self, distance, speed=0.3):
        """向左平移指定距离"""
        start_time = self.get_clock().now()
        duration = distance / speed
        
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < duration:
            msg = self.create_motion_msg(vel_y=speed)  # 正值表示向左移动
            self.motion_pub.publish(msg)
            time.sleep(0.1)  # 控制发布频率
            
        # 停止
        self.stop()
    
    def move_right(self, distance, speed=0.3):
        """向右平移指定距离"""
        start_time = self.get_clock().now()
        duration = distance / speed
        
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < duration:
            msg = self.create_motion_msg(vel_y=-speed)  # 负值表示向右移动
            self.motion_pub.publish(msg)
            time.sleep(0.1)  # 控制发布频率
            
        # 停止
        self.stop()
        
    def turn_right(self, angle, angular_speed=1.0):
        """右转指定角度（弧度）"""
        start_time = self.get_clock().now()
        duration = abs(angle) / angular_speed
        
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < duration:
            msg = self.create_motion_msg(vel_yaw=-angular_speed)  # 负值表示右转
            self.motion_pub.publish(msg)
            time.sleep(0.1)
            
        # 停止
        self.stop()

    
    def turn_left(self, angle, angular_speed=1.0):
        start_time = self.get_clock().now()
        duration = abs(angle) / angular_speed
        
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < duration:
            msg = self.create_motion_msg(vel_yaw=angular_speed)  # 负值表示右转
            self.motion_pub.publish(msg)
            time.sleep(0.1)
            
        # 停止
        self.stop()
        
    def stop(self):
        """停止运动"""
        msg = self.create_motion_msg()  # 所有速度为0
        self.motion_pub.publish(msg)
        time.sleep(0.5)  # 确保停止命令被执行
        
    def execute_motion_sequence(self):
        """执行运动序列"""
        try:
            # self.move_forward(0.1)  # 0.5米
            self.stand_up()
            time.sleep(2.8)  # 等待1秒确保完全停止

            # 1. 向前走50cm
            self.get_logger().info('Moving forward 50cm...')
            self.move_forward(1.8)  # 0.5米

            time.sleep(1.8)  # 等待1秒确保完全停止

            # 2. 右转90度
            self.get_logger().info('Turning right 90 degrees...')
            self.turn_right(1.1*math.pi)  # 90度 = π/2弧度
    
            time.sleep(3.0)  # 等待1秒确保完全停止

            # 3. 向前走100cm
            self.get_logger().info('Moving forward 100cm...')
            self.move_forward(2.0)  # 1.0米
    
            time.sleep(3.0)  # 等待1秒确保完全停止
            
            self.get_logger().info('Motion sequence completed!')

          
            self.get_logger().info('Motion sequence completed!')
    
            self.qr_client = self.create_client(Qrcode, 'Qrcode')
            
            # 等待服务可用
            if not self.qr_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error('QR码服务不可用')
                return
    
            self.get_logger().info('等待扫描QR码...')
            request = Qrcode.Request()
            future = self.qr_client.call_async(request)
            
            # 等待服务响应
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            
            if response.success:
                self.get_logger().info(f'检测到QR码: {response.content}')
                
                if response.content == 'A-1':
                    # 左转序列
                    self.get_logger().info('执行左转序列...')
                    self.turn_left(0.15*math.pi)
                    time.sleep(2.0)
                    
                    self.move_forward(2.0)
                    time.sleep(2.0)
    
                elif response.content == 'A-2':
                # 右转序列
                    self.get_logger().info('执行右转序列...')
                    self.turn_right(0.3*math.pi)
                    time.sleep(2.0)
                
                    self.move_forward(3.0)
                    time.sleep(2.0)
    
                    self.turn_left(0.4*math.pi)
    
                    self.move_right(0.8)
                    time.sleep(1.0)
    
                    self.move_backward(2.0, 0.8)
                    time.sleep(1.3)
                
                else:
                    self.get_logger().warn(f'未知的QR码内容: {response.content}')
                    return
                
            else:
                self.get_logger().error('QR码扫描失败')
                return




        except Exception as e:
            self.get_logger().error(f'Error during motion sequence: {str(e)}')
            self.stop()

def main(args=None):
    rclpy.init(args=args)
    node = DogMotionControl()
    
    try:
        # 等待一下确保节点完全初始化
        time.sleep(1.0)
        
        # 执行运动序列
        node.execute_motion_sequence()
        
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user')
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {str(e)}')
    finally:
        # 确保机器狗停止
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
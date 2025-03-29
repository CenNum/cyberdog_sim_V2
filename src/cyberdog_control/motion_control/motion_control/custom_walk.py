import rclpy
import json
import time
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult
from protocol.msg import MotionServoCmd

class DogController(Node):
    def __init__(self):
        super().__init__('dog_controller')
        
        # 参数声明
        self.declare_parameter(
            name='motion_sequence',
            value=[
                '["stop", 0.0, [0.0, 0.0, 0.0]]'
            ],
            descriptor=ParameterDescriptor(
                description='JSON格式运动序列，例: ["动作类型", 时长, [x速度,y速度,角速度]]',
                type=ParameterType.PARAMETER_STRING_ARRAY
            )
        )
        
        # 运动控制参数
        self.motion_sequence = []
        self.current_step = 0
        self.step_start_time = self.get_clock().now()
        self.is_running = False
        
        # 安全限制
        self.max_linear = 1.0  # m/s
        self.max_angular = 1.0  # rad/s
        
        # ROS2接口
        self.cmd_pub = self.create_publisher(MotionServoCmd, 'motion_servo_cmd', 10)
        self.control_timer = self.create_timer(0.02, self.control_loop)  # 50Hz
        self.add_on_set_parameters_callback(self.param_callback)
        
        # 初始化
        self.load_parameters()
        self.get_logger().info("节点已启动")

    def clamp_velocity(self, velocity):
        """速度限制"""
        return [
            max(min(velocity[0], self.max_linear), -self.max_linear),
            max(min(velocity[1], self.max_linear), -self.max_linear),
            max(min(velocity[2], self.max_angular), -self.max_angular)
        ]

    def load_parameters(self):
        """加载并验证运动序列参数"""
        try:
            raw_params = self.get_parameter('motion_sequence').value
            new_sequence = []
            
            for json_str in raw_params:
                step = json.loads(json_str)
                
                # 参数验证
                if len(step) != 3:
                    raise ValueError("每个步骤必须包含3个元素")
                
                step_config = {
                    'type': str(step[0]),
                    'duration': float(step[1]),
                    'velocity': self.clamp_velocity([float(v) for v in step[2]])
                }
                
                # 类型验证
                if step_config['type'] not in ['forward', 'backward', 'strafe', 'rotate', 'stop']:
                    raise ValueError(f"非法动作类型: {step_config['type']}")
                
                new_sequence.append(step_config)
            
            self.motion_sequence = new_sequence
            self.get_logger().info(f"成功加载 {len(self.motion_sequence)} 个运动步骤")
            
        except Exception as e:
            self.get_logger().error(f"参数错误: {str(e)}，使用默认配置")
            self.load_default_sequence()

    def load_default_sequence(self):
        """默认运动序列"""
        self.motion_sequence = [
            {'type': 'stop', 'duration': 0.0, 'velocity': [0.0, 0.0, 0.0]}
        ]

    def param_callback(self, params):
        """参数变更回调"""
        for param in params:
            if param.name == 'motion_sequence':
                self.load_parameters()
                self.current_step = 0
                self.step_start_time = self.get_clock().now()
                self.is_running = True
                return SetParametersResult(successful=True)
        return SetParametersResult(successful=True)

    def control_loop(self):
        """主控制循环"""
        if not self.motion_sequence or self.current_step >= len(self.motion_sequence):
            return

        current_step = self.motion_sequence[self.current_step]
        elapsed = (self.get_clock().now() - self.step_start_time).nanoseconds / 1e9

        # 持续发送控制指令
        self.send_command(current_step)
        
        # 检查步骤切换
        if elapsed >= current_step['duration']:
            self.current_step += 1
            if self.current_step < len(self.motion_sequence):
                self.step_start_time = self.get_clock().now()
                self.get_logger().info(f"执行步骤 {self.current_step+1}")
            else:
                self.shutdown_sequence()

    def send_command(self, step):
        """发送运动指令"""
        msg = MotionServoCmd()
        msg.motion_id = 303    # 慢速
        msg.cmd_type = 1       
        msg.vel_des = step['velocity']
        
        # 其他参数
        msg.pos_des = [0.0, 0.0, 0.0]
        msg.rpy_des = [0.0, 0.0, 0.0]
        msg.acc_des = [0.0, 0.0, 0.0]
        msg.step_height = [0.15, 0.15]
        
        self.cmd_pub.publish(msg)

    def shutdown_sequence(self):
        """安全停止序列"""
        self.get_logger().info("运动序列完成，等待新动作...")
        
        # 发送停止指令
        for _ in range(10):
            self.send_command({'velocity': [0.0, 0.0, 0.0]})
            time.sleep(0.01)
        
        # 关闭节点（可选）
        # self.destroy_node()
        # rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    try:
        controller = DogController()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
import json
import time
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult
from protocol.msg import MotionServoCmd

class DogController(Node):
    def __init__(self):
        super().__init__('dog_controller')
        
        # 参数声明（使用STRING类型）
        self.declare_parameter(
            name='motion_sequence',
            value='[]',  # 默认空JSON数组
            descriptor=ParameterDescriptor(
                description='JSON格式运动序列，例: [[ "forward",2.0,[0.5,0,0] ]]',
                type=ParameterType.PARAMETER_STRING
            )
        )
        
        # 运动控制参数
        self.motion_sequence = []
        self.current_step = 0
        self.step_start_time = None
        self.is_sequence_ready = False
        
        # 安全限制
        self.max_linear = 1.0  # m/s
        self.max_angular = 1.0  # rad/s
        
        # ROS2接口
        self.cmd_pub = self.create_publisher(MotionServoCmd, 'motion_servo_cmd', 10)
        self.control_timer = self.create_timer(0.02, self.control_loop)
        self.add_on_set_parameters_callback(self.param_callback)
        
        self.get_logger().info("节点已启动，等待运动序列参数...")
        self.load_parameters()  # 初始加载参数

    def clamp_velocity(self, velocity):
        """速度限制"""
        return [
            max(min(velocity[0], self.max_linear), -self.max_linear),
            max(min(velocity[1], self.max_linear), -self.max_linear),
            max(min(velocity[2], self.max_angular), -self.max_angular)
        ]
    
    def validate_sequence(self, raw_params):
        """校验整个运动序列"""
        if not isinstance(raw_params, list):
            raise ValueError("参数必须是JSON数组")
        
        sequence = []
        for idx, step in enumerate(raw_params):
            try:
                # 校验步骤结构
                if len(step) != 3:
                    raise ValueError(f"步骤{idx+1}需要3个参数（类型、时长、速度）")
                
                # 解析参数
                step_type = str(step[0]).lower()
                duration = float(step[1])
                velocity = [float(v) for v in step[2]]
                
                # 校验动作类型
                if step_type not in ['forward', 'backward', 'strafe', 'rotate', 'stop']:
                    raise ValueError(f"步骤{idx+1}无效动作类型: {step_type}")
                
                # 校验数值范围
                if duration < 0:
                    raise ValueError(f"步骤{idx+1}时长不能为负数")
                
                sequence.append({
                    'type': step_type,
                    'duration': duration,
                    'velocity': self.clamp_velocity(velocity)
                })
                
            except Exception as e:
                raise ValueError(f"步骤{idx+1}配置错误: {str(e)}")
        
        return sequence

    def load_parameters(self):
        """加载参数逻辑"""
        try:
            # 获取原始JSON字符串
            raw_json = self.get_parameter('motion_sequence').value
            self.get_logger().debug(f"原始参数值: {raw_json}")
            
            # 解析JSON
            parsed_data = json.loads(raw_json)
            self.get_logger().debug(f"解析后数据: {parsed_data}")
            
            # 验证数据结构
            self.motion_sequence = self.validate_sequence(parsed_data)
            
            if not self.motion_sequence:
                self.get_logger().warn("运动序列为空")
                self.is_sequence_ready = False
            else:
                self.is_sequence_ready = True
                self.get_logger().info(f"成功加载 {len(self.motion_sequence)} 个运动步骤")
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON解析失败: {str(e)}")
            self.is_sequence_ready = False
        except Exception as e:
            self.get_logger().error(f"参数错误: {str(e)}")
            self.is_sequence_ready = False

    def param_callback(self, params):
        """参数变更回调"""
        for param in params:
            if param.name == 'motion_sequence':
                self.load_parameters()
                if self.is_sequence_ready:
                    self.current_step = 0
                    self.step_start_time = self.get_clock().now()
                    self.get_logger().info(f"开始执行新运动序列进入步骤! 进入步骤{self.current_step+1}/{len(self.motion_sequence)}")
                return SetParametersResult(successful=True)
        return SetParametersResult(successful=True)

    def control_loop(self):
        """主控制循环"""
        # 只在有有效序列时执行
        if not self.is_sequence_ready or self.current_step >= len(self.motion_sequence):
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
                self.get_logger().info(f"进入步骤 {self.current_step+1}/{len(self.motion_sequence)}")
            else:
                self.shutdown_sequence()

    def send_command(self, step):
        """发送运动指令"""
        msg = MotionServoCmd()
        msg.motion_id = 303    # 慢速
        msg.cmd_type = 1       
        msg.vel_des = step['velocity']
        
        # 其他参数保持默认
        msg.pos_des = [0.0, 0.0, 0.0]
        msg.rpy_des = [0.0, 0.0, 0.0]
        msg.acc_des = [0.0, 0.0, 0.0]
        msg.step_height = [0.15, 0.15]
        
        self.cmd_pub.publish(msg)

    def shutdown_sequence(self):
        """安全停止序列"""
        self.get_logger().info("运动序列执行完成,等待下一个运动")
        self.is_sequence_ready = False
        
        # 发送3次停止指令确保停止
        for _ in range(3):
            self.send_command({'velocity': [0.0, 0.0, 0.0]})
            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    controller = DogController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
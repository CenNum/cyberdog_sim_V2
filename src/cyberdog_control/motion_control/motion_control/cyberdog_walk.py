import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter, ParameterType
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.qos import qos_profile_sensor_data
from protocol.msg import MotionServoCmd
from sensor_msgs.msg import Imu
from msgs_lane.srv import Qrcode
import math
import yaml

class DogControl(Node):
    def __init__(self):
        super().__init__('dog_control')
        
        # 参数声明
        self.declare_parameter(
            'motion_sequence',
            '',
            ParameterDescriptor(
                name='motion_sequence',
                type=ParameterType.PARAMETER_STRING,
                description='YAML格式的运动序列'
            )
        )
        self.add_on_set_parameters_callback(self.parameter_callback)

        # 控制指令发布
        self.publisher_ = self.create_publisher(MotionServoCmd, 'motion_servo_cmd', 10)
        
        # IMU数据订阅
        self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            qos_profile_sensor_data
        )
        
        # 控制定时器
        self.timer = self.create_timer(0.1, self.send_command)

        # 状态变量初始化
        self.is_sequence_ready = False
        self.motion_sequence = []
        self.current_motion_index = 0
        self.motion_start_time = self.get_clock().now()
        
        # 运动状态跟踪
        self.motion_start_yaw = None
        self.motion_start_body_x = None
        self.motion_start_body_y = None
        self.body_x = 0.0
        self.body_y = 0.0
        self.last_vx = 0.0
        self.last_vy = 0.0
        self.last_time = self.get_clock().now()
        self.current_yaw = 0.0

        # PID参数初始化
        self._init_pid_parameters()
        
        self.get_logger().info('节点已启动，等待运动序列参数...')

    def _init_pid_parameters(self):
        """初始化PID控制参数"""
        # 线性运动
        self.pid_kp_linear = 1.0
        self.pid_ki_linear = 0.0
        self.pid_kd_linear = 0.1
        self.linear_pid_integral = 0.0
        self.linear_pid_last_error = 0.0

        # 平移运动
        self.pid_kp_strafe = 1.0
        self.pid_ki_strafe = 0.0
        self.pid_kd_strafe = 0.1
        self.strafe_pid_integral = 0.0
        self.strafe_pid_last_error = 0.0

        # 转向
        self.pid_kp_turn = 1.0
        self.pid_ki_turn = 0.0
        self.pid_kd_turn = 0.1
        self.turn_pid_integral = 0.0
        self.turn_pid_last_error = 0.0

    def parameter_callback(self, params):
        """参数更新回调"""
        for param in params:
            if param.name == 'motion_sequence' and param.type_ == Parameter.Type.STRING:
                if not self._validate_motion_parameter(param.value):
                    return SetParametersResult(
                        successful=False,
                        reason="Invalid motion sequence format"
                    )
                
                try:
                    self.motion_sequence = yaml.safe_load(param.value)
                    self.current_motion_index = 0
                    self.is_sequence_ready = True
                    self.motion_start_time = self.get_clock().now()
                    self._reset_state_variables()
                    
                    self.get_logger().info(
                        f"接收到新运动序列，共{len(self.motion_sequence)}个动作\n" +
                        yaml.dump(self.motion_sequence, allow_unicode=True)
                    )
                    return SetParametersResult(successful=True)
                
                except Exception as e:
                    self.get_logger().error(f"参数处理失败: {str(e)}")
                    return SetParametersResult(
                        successful=False,
                        reason=f"Parameter processing failed: {str(e)}"
                    )
        return SetParametersResult(successful=True)

    def _validate_motion_parameter(self, value):
        """验证运动序列参数有效性"""
        try:
            sequence = yaml.safe_load(value)
            return self._validate_sequence(sequence)
        except:
            return False

    def _validate_sequence(self, sequence):
        """验证运动序列格式"""
        if not isinstance(sequence, list):
            return False
        
        valid_types = ['wait', 'forward', 'turn', 'strafe']
        for motion in sequence:
            if not isinstance(motion, dict):
                return False
            if 'type' not in motion:
                return False
            if motion['type'] not in valid_types:
                return False
            
            # 参数完整性检查
            if motion['type'] == 'wait' and 'duration' not in motion:
                return False
            if motion['type'] == 'forward' and ('distance' not in motion or 'velocity' not in motion):
                return False
            if motion['type'] == 'turn' and 'angle' not in motion:
                return False
            if motion['type'] == 'strafe' and ('distance' not in motion or 'velocity' not in motion or 'direction' not in motion):
                return False
        return True

    def _reset_state_variables(self):
        """重置所有状态变量"""
        self.motion_start_yaw = None
        self.motion_start_body_x = None
        self.motion_start_body_y = None
        self.body_x = 0.0
        self.body_y = 0.0
        self._init_pid_parameters()

    def imu_callback(self, msg):
        """IMU数据处理"""
        q = msg.orientation
        # 四元数转偏航角（绕Z轴旋转）
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def send_command(self):
        """主控制循环（修改关键部分）"""
        # 只在有有效运动序列时发送指令
        if not self.is_sequence_ready:
            return  # 直接返回不发送任何指令

        cmd = MotionServoCmd()
        cmd.motion_id = 308
        cmd.cmd_type = 1
        cmd.vel_des = [0.0, 0.0, 0.0]
        cmd.step_height = [0.15, 0.15]

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        elapsed = (now - self.motion_start_time).nanoseconds * 1e-9
        self.last_time = now

        # 更新位置估计
        self.body_x += self.last_vx * dt
        self.body_y += self.last_vy * dt

        if self.current_motion_index < len(self.motion_sequence):
            self._execute_current_motion(cmd, now, elapsed, dt)
        else:
            self.get_logger().info("运动序列执行完成")
            self.is_sequence_ready = False  # 标记序列完成
            return  # 完成时直接返回不发送指令

        self.last_vx = cmd.vel_des[0]
        self.last_vy = cmd.vel_des[1]
        self.publisher_.publish(cmd)  # 只有需要运动时才发布指令

    def _execute_current_motion(self, cmd, now, elapsed, dt):
        motion = self.motion_sequence[self.current_motion_index]
        
        try:
            if motion['type'] == 'wait':
                self._handle_wait(motion, elapsed, now)
            elif motion['type'] == 'forward':
                self._handle_linear_motion(motion, cmd, dt)
            elif motion['type'] == 'turn':
                self._handle_turn(motion, cmd, dt)
            elif motion['type'] == 'strafe':
                self._handle_strafe(motion, cmd, dt)
        except KeyError as e:
            self.get_logger().error(f"运动参数错误: {str(e)}")
            self.current_motion_index += 1

    # 以下处理函数保持不变
    def _handle_wait(self, motion, elapsed, now):
        if elapsed >= motion['duration']:
            self.current_motion_index += 1
            self.motion_start_time = now
            self.get_logger().info(f"等待完成 ({motion['duration']}s)")

    def _handle_linear_motion(self, motion, cmd, dt):
        if self.motion_start_body_x is None:
            self.motion_start_body_x = self.body_x
            self.linear_pid_integral = 0.0

        target = motion['distance']
        current = self.body_x - self.motion_start_body_x
        error = target - current

        # PID计算
        self.linear_pid_integral += error * dt
        derivative = (error - self.linear_pid_last_error) / dt if dt > 0 else 0.0
        velocity = (self.pid_kp_linear * error +
                   self.pid_ki_linear * self.linear_pid_integral +
                   self.pid_kd_linear * derivative)
        
        # 速度限幅
        velocity = max(-motion['velocity'], min(motion['velocity'], velocity))
        cmd.vel_des[0] = velocity
        self.linear_pid_last_error = error

        # 完成条件
        if abs(error) < 0.03:
            self.current_motion_index += 1
            self.motion_start_body_x = None
            self.get_logger().info(f"前进完成 (距离: {current:.2f}m)")

    def _handle_turn(self, motion, cmd, dt):
        if self.current_yaw is None:
            return

        if self.motion_start_yaw is None:
            self.motion_start_yaw = self.current_yaw
            self.turn_pid_integral = 0.0

        target_yaw = self._normalize_angle(self.motion_start_yaw + motion['angle'])
        current_yaw = self._normalize_angle(self.current_yaw)
        error = self._normalize_angle(target_yaw - current_yaw)

        # PID计算
        self.turn_pid_integral += error * dt
        derivative = (error - self.turn_pid_last_error) / dt if dt > 0 else 0.0
        angular_vel = (self.pid_kp_turn * error +
                      self.pid_ki_turn * self.turn_pid_integral +
                      self.pid_kd_turn * derivative)
        
        # 角速度限幅
        angular_vel = max(-1.0, min(1.0, angular_vel))
        cmd.vel_des[2] = angular_vel
        self.turn_pid_last_error = error

        # 完成条件
        if abs(error) < 0.05:  # 约2.86度
            self.current_motion_index += 1
            self.motion_start_yaw = None
            self.get_logger().info(f"转向完成 (当前偏航角: {math.degrees(current_yaw):.1f}°)")

    def _handle_strafe(self, motion, cmd, dt):
        if self.motion_start_body_y is None:
            self.motion_start_body_y = self.body_y
            self.strafe_pid_integral = 0.0

        direction = -1 if motion['direction'] == 'right' else 1
        target = motion['distance'] * direction
        current = self.body_y - self.motion_start_body_y
        error = target - current

        # PID计算
        self.strafe_pid_integral += error * dt
        derivative = (error - self.strafe_pid_last_error) / dt if dt > 0 else 0.0
        velocity = (self.pid_kp_strafe * error +
                   self.pid_ki_strafe * self.strafe_pid_integral +
                   self.pid_kd_strafe * derivative)
        
        # 速度限幅
        velocity = max(-motion['velocity'], min(motion['velocity'], velocity))
        cmd.vel_des[1] = velocity
        self.strafe_pid_last_error = error

        # 完成条件
        if abs(error) < 0.03:
            self.current_motion_index += 1
            self.motion_start_body_y = None
            self.get_logger().info(f"平移完成 (方向: {motion['direction']}, 距离: {abs(current):.2f}m)")

    @staticmethod
    def _normalize_angle(angle):
        """将角度规范到[-π, π]范围"""
        return (angle + math.pi) % (2 * math.pi) - math.pi

def main(args=None):
    rclpy.init(args=args)
    controller = DogControl()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("节点被手动关闭")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



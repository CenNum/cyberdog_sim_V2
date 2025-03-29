import rclpy
from rclpy.node import Node
from protocol.msg import MotionServoCmd

class DogControl(Node):
    def __init__(self):
        super().__init__('dog_control')
        self.publisher_ = self.create_publisher(MotionServoCmd, 'motion_servo_cmd', 10)
        self.timer = self.create_timer(0.1, self.send_command)
        
        # 可调参数配置（单位：秒）
        self.forward1_duration = 3.4    # 第一阶段前进时间 x
        self.turn_duration = 3.8     # 右转90度时间（π/2÷0.5 rad/s）
        self.forward2_duration = 4    # 第二阶段前进时间 y
        self.strafe_duration = 3      # 右平移时间 z
        self.backward_duration = 1.8    # 后退时间 w
        
        # 状态机初始化
        self.state_machine = [
            'forward1', 
            'turn_right',
            'forward2',
            'strafe_right',
            'backward',
            'stop'
        ]
        self.current_state = 0
        self.state_start_time = self.get_clock().now()

    def send_command(self):
        msg = MotionServoCmd()
        msg.motion_id = 303
        msg.cmd_type = 1
        msg.cmd_source = 3
        msg.step_height = [0.2, 0.2]

        # 计算当前状态持续时间
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.state_start_time).nanoseconds / 1e9
        state_name = self.state_machine[self.current_state]

        # 状态控制逻辑
        if state_name == 'forward1':
            msg.vel_des = [0.5, 0.0, 0.0]  # 前进
            if elapsed_time >= self.forward1_duration:
                self.next_state()
                
        elif state_name == 'turn_right':
            msg.vel_des = [0.0, 0.0, -0.5]  # 右转
            if elapsed_time >= self.turn_duration:
                self.next_state()
                
        elif state_name == 'forward2':
            msg.vel_des = [0.5, 0.0, 0.0]  # 继续前进
            if elapsed_time >= self.forward2_duration:
                self.next_state()
                
        elif state_name == 'strafe_right':
            msg.vel_des = [0.0, -0.5, 0.0]  # 右侧平移
            if elapsed_time >= self.strafe_duration:
                self.next_state()
                
        elif state_name == 'backward':
            msg.vel_des = [-0.5, 0.0, 0.0]  # 后退
            if elapsed_time >= self.backward_duration:
                self.next_state()
                
        elif state_name == 'stop':
            msg.vel_des = [0.0, 0.0, 0.0]  # 停止
            self.timer.cancel()
            self.get_logger().info('Movement completed')

        # 设置其他固定参数
        msg.rpy_des = [0.0, 0.0, 0.0]
        msg.pos_des = [0.0, 0.0, 0.0]
        msg.acc_des = [0.0, 0.0, 0.0]
        msg.ctrl_point = [0.0, 0.0, 0.0]
        msg.foot_pose = [0.0, 0.0, 0.0]

        self.publisher_.publish(msg)
        self.get_logger().info(f'Current State: {state_name}')

    def next_state(self):
        if self.current_state < len(self.state_machine)-1:
            self.current_state += 1
            self.state_start_time = self.get_clock().now()
            self.get_logger().info(f'Transition to {self.state_machine[self.current_state]}')

def main(args=None):
    rclpy.init(args=args)
    node = DogControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
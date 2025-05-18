import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from protocol.msg import MotionServoCmd

class DogNavigationControl(Node):
    def __init__(self):
        super().__init__('dog_navigation_control')

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)

        self.publisher_ = self.create_publisher(
            MotionServoCmd,
            'motion_servo_cmd',
            10)

        self.timer = self.create_timer(0.05, self.send_command)  # 20Hz

        self.current_vel = [0.0, 0.0, 0.0]  # [x, y, theta]

        # 参数：最大角速度、角速度缩放系数、转弯减速系数
        self.max_ang_vel = 1.0           # 限幅最大角速度
        self.ang_scale = 0.5             # 角速度缩放因子
        self.turn_slowdown_factor = 0.1  # 转弯时线速度缩放因子
        self.turn_threshold = 0.2        # 超过该角速度时减速

        self.get_logger().info('DogNavigationControl node has been started.')

    def create_motion_msg(self, vel_x=0.0, vel_y=0.0, vel_yaw=0.0):
        msg = MotionServoCmd()
        msg.motion_id = 303
        msg.value = 2
        msg.cmd_type = 1
        msg.cmd_source = 2
        msg.vel_des = [vel_x, vel_y, vel_yaw]
        msg.step_height = [0.2, 0.2]
        return msg

    def cmd_vel_callback(self, msg):
        # 限制角速度 [-max, max] 并缩放
        raw_ang = msg.angular.z
        limited_ang = max(min(raw_ang, self.max_ang_vel), -self.max_ang_vel)
        scaled_ang = limited_ang * self.ang_scale
        self.current_vel[2] = scaled_ang

        # 判断是否需要转弯减速
        if abs(scaled_ang) > self.turn_threshold:
            self.current_vel[0] = msg.linear.x * self.turn_slowdown_factor
            self.current_vel[1] = msg.linear.y * self.turn_slowdown_factor
            mode = "转弯减速"
        else:
            self.current_vel[0] = msg.linear.x
            self.current_vel[1] = msg.linear.y
            mode = "正常行走"

        self.get_logger().info(
            f'[接收cmd_vel] x={msg.linear.x:.2f}, y={msg.linear.y:.2f}, θ={msg.angular.z:.2f} '
            f'-> 输出: x={self.current_vel[0]:.2f}, y={self.current_vel[1]:.2f}, θ={scaled_ang:.2f} [{mode}]')

    def send_command(self):
        msg = self.create_motion_msg(
            vel_x=self.current_vel[0],
            vel_y=self.current_vel[1],
            vel_yaw=self.current_vel[2]
        )
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DogNavigationControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

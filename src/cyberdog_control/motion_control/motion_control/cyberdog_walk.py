import rclpy
from rclpy.node import Node
from protocol.msg import MotionServoCmd
from sensor_msgs.msg import Imu
import math
from rclpy.qos import qos_profile_sensor_data

class DogControl(Node):

    def __init__(self):
        super().__init__('dog_control')
        # 发布控制指令
        self.publisher_ = self.create_publisher(MotionServoCmd, 'motion_servo_cmd', 10)
        # 订阅 /imu 数据，使用 sensor_data 预定义的 QoS 策略
        self.create_subscription(Imu, '/imu', self.imu_callback, qos_profile_sensor_data)
        # 定时器，每 0.1s 调用一次 send_command
        self.timer = self.create_timer(0.1, self.send_command)

        # 定义运动序列，支持 "wait"、"forward"（前进）、"turn"（转向）、"strafe"（平移）
        # wait：需要 "duration"（等待时间，单位秒）
        # forward：需要 "distance"（米） 和 "velocity"（m/s）
        # turn：需要 "angle"（弧度，正值左转，负值右转）
        # strafe：需要 "distance"（米）、"velocity"（m/s） 和 "direction"（"right" 或 "left"）
        self.motion_sequence = [
            {"type": "wait", "duration": 5.0},              # 等待5秒，确保机器狗站起来
            {"type": "forward", "distance": 1.0, "velocity": 1.0},   
            {"type": "turn", "angle": -math.pi/1.9}, 
            {"type": "forward", "distance": 1.3, "velocity": 1.0}
            #{"type": "strafe", "distance": 0.5, "velocity": 0.5, "direction": "right"}
        ]
        self.current_motion_index = 0
        self.motion_start_time = self.get_clock().now()
        self.motion_start_yaw = None  # 转向运动时记录起始 yaw

        # PID 控制参数（用于转向）
        self.pid_kp = 1.0
        self.pid_ki = 0.0
        self.pid_kd = 0.1
        self.pid_integral = 0.0
        self.pid_last_error = 0.0

        self.current_yaw = None

        self.get_logger().info('DogControl node has been started.')

    def imu_callback(self, msg: Imu):
        # 将四元数转换为 yaw 角
        q = msg.orientation
        yaw = math.atan2(2*(q.w * q.z + q.x * q.y), 1 - 2*(q.y * q.y + q.z * q.z))
        self.current_yaw = yaw

    def normalize_angle(self, angle):
        """将角度归一化到 [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def send_command(self):
        now = self.get_clock().now()
        elapsed = (now - self.motion_start_time).nanoseconds * 1e-9  # 单位秒

        cmd = MotionServoCmd()
        # 设置基础参数
        cmd.motion_id = 308
        #cmd.value = 2
        cmd.cmd_type = 1
        #cmd.cmd_source = 3
        cmd.rpy_des = [0.0, 0.0, 0.0]
        cmd.pos_des = [0.0, 0.0, 0.0]
        cmd.acc_des = [0.0, 0.0, 0.0]
        cmd.ctrl_point = [0.0, 0.0, 0.0]
        cmd.foot_pose = [0.0, 0.0, 0.0]
        cmd.step_height = [0.2, 0.2]

        if self.current_motion_index >= len(self.motion_sequence):
            # 所有运动完成，停止运动
            cmd.vel_des = [0.0, 0.0, 0.0]
            self.get_logger().info("All motions completed. Stopped.")
        else:
            current_motion = self.motion_sequence[self.current_motion_index]
            if current_motion["type"] == "wait":
                # 等待运动：仅等待指定时间，不发出运动指令
                self.get_logger().info(
                    f"Motion {self.current_motion_index} [Wait]: Elapsed {elapsed:.2f}s / Target {current_motion['duration']:.2f}s"
                )
                cmd.vel_des = [0.0, 0.0, 0.0]
                if elapsed >= current_motion["duration"]:
                    self.get_logger().info("Wait motion completed.")
                    self.current_motion_index += 1
                    self.motion_start_time = now

            elif current_motion["type"] == "forward":
                # 前进运动：目标时间 = 距离 / 速度
                target_duration = current_motion["distance"] / current_motion["velocity"]
                cmd.vel_des = [current_motion["velocity"], 0.0, 0.0]
                self.get_logger().info(
                    f"Motion {self.current_motion_index} [Forward]: Elapsed {elapsed:.2f}s / Target {target_duration:.2f}s"
                )
                if elapsed >= target_duration:
                    self.get_logger().info("Forward motion completed.")
                    self.current_motion_index += 1
                    self.motion_start_time = now

            elif current_motion["type"] == "strafe":
                # 平移运动：目标时间 = 距离 / 速度
                target_duration = current_motion["distance"] / current_motion["velocity"]
                # 根据方向设置 y 轴速度；ROS 标准坐标系中，y 轴正方向指向左侧
                if current_motion.get("direction", "right") == "right":
                    y_vel = -current_motion["velocity"]
                else:
                    y_vel = current_motion["velocity"]
                cmd.vel_des = [0.0, y_vel, 0.0]
                self.get_logger().info(
                    f"Motion {self.current_motion_index} [Strafe]: Elapsed {elapsed:.2f}s / Target {target_duration:.2f}s"
                )
                if elapsed >= target_duration:
                    self.get_logger().info("Strafe motion completed.")
                    self.current_motion_index += 1
                    self.motion_start_time = now

            elif current_motion["type"] == "turn":
                # 转向运动：使用 PID 控制
                if self.motion_start_yaw is None and self.current_yaw is not None:
                    self.motion_start_yaw = self.current_yaw
                    # 重置 PID 状态
                    self.pid_integral = 0.0
                    self.pid_last_error = 0.0
                    self.get_logger().info(f"Turn motion started. Start yaw: {self.motion_start_yaw:.2f} rad")

                if self.motion_start_yaw is None or self.current_yaw is None:
                    self.get_logger().warn("Turn: Waiting for IMU data...")
                    return

                # 目标 yaw = 起始 yaw + 目标转角
                desired_yaw = self.normalize_angle(self.motion_start_yaw + current_motion["angle"])
                error = self.normalize_angle(desired_yaw - self.current_yaw)
                self.pid_integral += error * 0.1  # dt = 0.1s
                derivative = (error - self.pid_last_error) / 0.1
                self.pid_last_error = error

                # PID 控制计算角速度命令
                angular_velocity_command = self.pid_kp * error + self.pid_ki * self.pid_integral + self.pid_kd * derivative

                # 限制角速度范围
                max_angular_velocity = 0.5  # rad/s
                angular_velocity_command = max(-max_angular_velocity, min(max_angular_velocity, angular_velocity_command))

                cmd.vel_des = [0.0, 0.0, angular_velocity_command]
                self.get_logger().info(
                    f"Motion {self.current_motion_index} [Turn]: Target yaw: {desired_yaw:.2f} rad, "
                    f"Current yaw: {self.current_yaw:.2f} rad, Error: {error:.2f} rad, "
                    f"Angular vel: {angular_velocity_command:.2f} rad/s"
                )

                # 当误差足够小时，认为转向完成
                if abs(error) < 0.05:
                    self.get_logger().info("Turn motion completed.")
                    self.current_motion_index += 1
                    self.motion_start_time = now
                    self.motion_start_yaw = None
                    self.pid_integral = 0.0
                    self.pid_last_error = 0.0

            else:
                self.get_logger().warn("Unknown motion type, skipping.")
                self.current_motion_index += 1

        self.publisher_.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = DogControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

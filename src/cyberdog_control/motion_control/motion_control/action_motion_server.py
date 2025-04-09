# 已知bug:
# 1. 动作服务器未能正确处理中止请求，导致后续动作无法执行。
import rclpy
import asyncio
import math
import yaml
import traceback
from threading import Lock
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from action_msgs.msg import GoalStatus
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from protocol.msg import MotionServoCmd
from cyberdog_interfaces.action import MotionSequence


def run_coroutine(coro):
    """启动一个新的事件循环以运行协程"""
    try:
        loop = asyncio.get_event_loop()
    except RuntimeError:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
    return loop.run_until_complete(coro)


class DogControlAction(Node):
    def __init__(self):
        super().__init__('dog_control_action')
        self._lock = asyncio.Lock()

        # 动作服务器
        self._action_server = ActionServer(
            self,
            MotionSequence,
            'execute_motion_sequence',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup()
        )

        self.cmd_publisher = self.create_publisher(MotionServoCmd, 'motion_servo_cmd', 10)

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            qos_profile_sensor_data
        )

        self._current_goal = None
        self._is_executing = False
        self._current_step = 0
        self._motion_sequence = []

        self._start_yaw = 0.0
        self._start_x = 0.0
        self._start_y = 0.0
        self._current_x = 0.0
        self._current_y = 0.0
        self._last_vx = 0.0
        self._last_vy = 0.0
        self._last_time = self.get_clock().now()
        self._current_yaw = 0.0
        self._motion_start_time = self.get_clock().now()

        self._init_pid_controllers()

        self.get_logger().info('动作服务器已启动，等待指令...')

    def _init_pid_controllers(self):
        self.linear_kp = 1.2
        self.linear_ki = 0.01
        self.linear_kd = 0.2
        self.linear_integral = 0.0
        self.linear_last_error = 0.0

        self.strafe_kp = 1.0
        self.strafe_ki = 0.01
        self.strafe_kd = 0.15
        self.strafe_integral = 0.0
        self.strafe_last_error = 0.0

        self.yaw_kp = 1.5
        self.yaw_ki = 0.01
        self.yaw_kd = 0.3
        self.yaw_integral = 0.0
        self.yaw_last_error = 0.0

    def execute_callback(self, goal_handle: ServerGoalHandle):
        """用run_coroutine包装异步回调"""
        return run_coroutine(self._execute_callback_async(goal_handle))

    async def _execute_callback_async(self, goal_handle: ServerGoalHandle):
        self.get_logger().info('接收到新动作请求')
        result = MotionSequence.Result()
        success = False

        try:
            async with self._lock:
                if not goal_handle.is_active:
                    self.get_logger().warn('目标已失效')
                    result.success = False
                    return result

                if self._is_executing:
                    if self._current_goal and self._current_goal.is_active:
                        self.get_logger().warn('中止前序动作')
                        try:
                            self._current_goal.abort()
                            await asyncio.sleep(0.1)
                        except Exception as e:
                            self.get_logger().error(f'中止失败: {str(e)}')

                if goal_handle.status == GoalStatus.STATUS_ACCEPTED:
                    goal_handle.execute()
                    self.get_logger().debug('状态转换: ACCEPTED -> EXECUTING')

                self._is_executing = True
                self._current_goal = goal_handle

                sequence = yaml.safe_load(goal_handle.request.motion_sequence)
                if not self._validate_sequence(sequence):
                    raise ValueError("无效动作序列")

                self._motion_sequence = sequence
                self._current_step = 0
                self._reset_state()
                self._motion_start_time = self.get_clock().now()

                success = True

                while rclpy.ok() and self._current_step < len(self._motion_sequence):
                    if not goal_handle.is_active:
                        self.get_logger().warn('目标状态异常')
                        success = False
                        break

                    cmd, done = self._execute_step()
                    if cmd:
                        self.cmd_publisher.publish(cmd)

                    feedback = MotionSequence.Feedback()
                    feedback.current_step = self._current_step + 1
                    feedback.total_steps = len(self._motion_sequence)
                    goal_handle.publish_feedback(feedback)

                    if done:
                        self._current_step += 1
                        self._motion_start_time = self.get_clock().now()

                    await asyncio.sleep(0.1)

        except yaml.YAMLError:
            self.get_logger().error(f'YAML错误: {traceback.format_exc()}')
            success = False
        except Exception:
            self.get_logger().error(f'执行异常: {traceback.format_exc()}')
            success = False
        finally:
            result.success = success
            try:
                if goal_handle.is_active:
                    if success:
                        goal_handle.succeed()
                        self.get_logger().info('✅ 执行成功')
                    else:
                        goal_handle.abort()
                        self.get_logger().warn('❌ 执行失败')
            except Exception as e:
                self.get_logger().error(f'状态更新失败: {str(e)}')
            await self._cleanup()

        return result

    def _execute_step(self):
        """执行单个步骤"""
        if self._current_step >= len(self._motion_sequence):
            return None, True
        
        current_motion = self._motion_sequence[self._current_step]
        cmd = MotionServoCmd()
        cmd.motion_id = 308
        cmd.cmd_type = 1
        cmd.vel_des = [0.0, 0.0, 0.0]
        cmd.step_height = [0.15, 0.15]
        
        done = False
        try:
            motion_type = current_motion['type']
            now = self.get_clock().now()
            dt = (now - self._last_time).nanoseconds * 1e-9
            elapsed = (now - self._motion_start_time).nanoseconds * 1e-9

            # 更新位置估计
            self._current_x += self._last_vx * dt
            self._current_y += self._last_vy * dt

            if motion_type == 'wait':
                done = elapsed >= current_motion['duration']
            elif motion_type == 'forward':
                done = self._handle_linear(current_motion, cmd, dt)
            elif motion_type == 'turn':
                done = self._handle_turn(current_motion, cmd, dt)
            elif motion_type == 'strafe':
                done = self._handle_strafe(current_motion, cmd, dt)
            else:
                self.get_logger().warn(f'未知动作类型: {motion_type}')
                done = True

            self._last_time = now
            self._last_vx = cmd.vel_des[0]
            self._last_vy = cmd.vel_des[1]

        except KeyError as e:
            self.get_logger().error(f'缺少参数: {str(e)}')
            done = True
        
        return cmd, done

    def _handle_linear(self, motion, cmd, dt):
        """处理直线运动"""
        if self._start_x is None:
            self._start_x = self._current_x
            self.linear_integral = 0.0
            self.linear_last_error = 0.0
        
        target = motion['distance']
        current = self._current_x - self._start_x
        error = target - current
        
        # PID计算
        self.linear_integral += error * dt
        derivative = (error - self.linear_last_error) / dt if dt > 0 else 0.0
        velocity = (self.linear_kp * error +
                   self.linear_ki * self.linear_integral +
                   self.linear_kd * derivative)
        
        # 限幅
        max_speed = motion.get('velocity', 0.5)
        velocity = max(-max_speed, min(velocity, max_speed))
        cmd.vel_des[0] = velocity
        self.linear_last_error = error
        
        # 完成判断
        if abs(error) < 0.03:
            self.get_logger().info(f"前进完成 目标: {target:.2f}m, 实际: {current:.2f}m")
            self._start_x = None
            return True
        return False

    def _handle_turn(self, motion, cmd, dt):
        """处理旋转动作"""
        if self._start_yaw is None:
            self._start_yaw = self._current_yaw
            self.yaw_integral = 0.0
            self.yaw_last_error = 0.0
        
        target = self._normalize_angle(self._start_yaw + motion['angle'])
        current = self._normalize_angle(self._current_yaw)
        error = self._normalize_angle(target - current)
        
        # PID计算
        self.yaw_integral += error * dt
        derivative = (error - self.yaw_last_error) / dt if dt > 0 else 0.0
        angular = (self.yaw_kp * error +
                  self.yaw_ki * self.yaw_integral +
                  self.yaw_kd * derivative)
        
        # 限幅
        angular = max(-1.0, min(angular, 1.0))
        cmd.vel_des[2] = angular
        self.yaw_last_error = error
        
        # 完成判断
        if abs(error) < 0.05:
            self.get_logger().info(f"转向完成 目标: {math.degrees(target):.1f}°, 当前: {math.degrees(current):.1f}°")
            self._start_yaw = None
            return True
        return False

    def _handle_strafe(self, motion, cmd, dt):
        """处理平移动作"""
        if self._start_y is None:
            self._start_y = self._current_y
            self.strafe_integral = 0.0
            self.strafe_last_error = 0.0
        
        direction = -1 if motion['direction'] == 'right' else 1
        target = motion['distance'] * direction
        current = self._current_y - self._start_y
        error = target - current
        
        # PID计算
        self.strafe_integral += error * dt
        derivative = (error - self.strafe_last_error) / dt if dt > 0 else 0.0
        velocity = (self.strafe_kp * error +
                   self.strafe_ki * self.strafe_integral +
                   self.strafe_kd * derivative)
        
        # 限幅
        max_speed = motion.get('velocity', 0.4)
        velocity = max(-max_speed, min(velocity, max_speed))
        cmd.vel_des[1] = velocity
        self.strafe_last_error = error
        
        # 完成判断
        if abs(error) < 0.03:
            self.get_logger().info(f"平移完成 方向: {motion['direction']}, 距离: {abs(current):.2f}m")
            self._start_y = None
            return True
        return False

    def imu_callback(self, msg):
        """处理IMU数据"""
        q = msg.orientation
        # 四元数转偏航角
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
        self._current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _validate_sequence(self, sequence):
        """验证运动序列格式"""
        if not isinstance(sequence, list):
            return False
        valid_types = ['wait', 'forward', 'turn', 'strafe']
        for motion in sequence:
            if not isinstance(motion, dict) or 'type' not in motion:
                return False
            if motion['type'] not in valid_types:
                return False
            # 参数检查
            if motion['type'] == 'wait' and not isinstance(motion.get('duration', 0), (int, float)):
                return False
            if motion['type'] == 'forward' and not all(isinstance(motion.get(k, 0), (int, float)) for k in ['distance', 'velocity']):
                return False
            if motion['type'] == 'turn' and not isinstance(motion.get('angle', 0), (int, float)):
                return False
            if motion['type'] == 'strafe' and (not all(isinstance(motion.get(k, 0), (int, float)) for k in ['distance', 'velocity']) or motion.get('direction', '') not in ['left', 'right']):
                return False
        return True


    async def _cleanup(self):
        """异步清理资源"""
        async with self._lock:  # 使用异步锁保护清理过程
            # 发送停止指令
            stop_cmd = MotionServoCmd()
            stop_cmd.motion_id = 308
            stop_cmd.cmd_type = 1
            stop_cmd.vel_des = [0.0, 0.0, 0.0]
            self.cmd_publisher.publish(stop_cmd)
            
            # 重置状态
            self._is_executing = False
            self._current_goal = None
            self._current_step = 0
            self._reset_state()
            self.get_logger().debug('资源清理完成')

    def _reset_state(self):
        """重置控制状态"""
        self._start_x = None
        self._start_y = None
        self._start_yaw = None
        self._current_x = 0.0
        self._current_y = 0.0
        self._init_pid_controllers()
        self._motion_start_time = self.get_clock().now()

    @staticmethod
    def _normalize_angle(angle):
        """角度标准化到[-π, π]"""
        return (angle + math.pi) % (2 * math.pi) - math.pi


def main(args=None):
    rclpy.init(args=args)
    controller = DogControlAction()
    executor = MultiThreadedExecutor()
    executor.add_node(controller)
    try:
        executor.spin()
    except KeyboardInterrupt:
        controller.get_logger().info("节点关闭中...")
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

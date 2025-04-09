#已知问题：
# 1. 动作序列执行超时后，程序不会退出，需要手动Ctrl+C退出。
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import yaml
from cyberdog_interfaces.action import MotionSequence

class MotionClient(Node):
    def __init__(self):
        super().__init__('motion_client')
        self._action_client = ActionClient(self, MotionSequence, 'execute_motion_sequence')
        self._timeout_timer = None

    def send_goal(self, sequence):
        goal_msg = MotionSequence.Goal()
        goal_msg.motion_sequence = yaml.dump(sequence)
        
        self._action_client.wait_for_server()
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
        # 设置30秒超时
        self._timeout_timer = self.create_timer(30.0, self.timeout_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('目标被拒绝')
            return

        self.get_logger().info('目标已接受，开始执行...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'进度: {feedback.current_step}/{feedback.total_steps}'
        )

    def get_result_callback(self, future):
        self._timeout_timer.cancel()
        try:
            result = future.result().result
            if result.success:
                self.get_logger().info('✅ 动作序列执行成功')
            else:
                self.get_logger().error('❌ 执行失败（服务端错误）')
        except Exception as e:
            self.get_logger().error(f'获取结果异常: {str(e)}')
        finally:
            rclpy.shutdown()

    def timeout_callback(self):
        self.get_logger().error('⏰ 动作执行超时')
        self._timeout_timer.cancel()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    # 示例运动序列1
    test_sequence1 = [
        {'type': 'wait', 'duration': 3.0},
        {'type': 'turn', 'angle': 0.08},  # 10度
        {'type': 'forward', 'distance': 0.99, 'velocity': 0.5},
        {'type': 'turn', 'angle': -1.59},  # 90度
        {'type': 'forward', 'distance': 1.2, 'velocity': 0.5},
    ]
    # 示例运动序列2
    test_sequence2 = [
        {'type': 'forward', 'distance': 0.4, 'velocity': 0.5},
        {'type': 'turn', 'angle': 0.6},
        {'type': 'forward', 'distance': 1.0, 'velocity': 0.5},
        {'type': 'turn', 'angle': -0.6},
        {'type': 'strafe', 'direction': 'left', 'distance': 0.38, 'velocity': 0.3},
        {'type': 'forward', 'distance': -1.2, 'velocity': 0.5},
    ]
    
    client = MotionClient()
    client.send_goal(test_sequence1)
    #client.send_goal(test_sequence2)
    rclpy.spin(client)

if __name__ == '__main__':
    main()

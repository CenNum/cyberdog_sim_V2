import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import yaml
import time
from cyberdog_interfaces.action import MotionSequence

class SyncMotionClient(Node):
    def __init__(self):
        super().__init__('sync_motion_client')
        self._action_client = ActionClient(self, MotionSequence, 'execute_motion_sequence')
        self.primary_done = False       # 主序列完成标志
        self.primary_success = False    # 主序列执行结果
        self.secondary_sent = False    # 次序列发送标志

    def send_primary(self):
        """ 发送主序列 """
        sequence = [
            {'type': 'wait', 'duration': 3.0},
            {'type': 'turn', 'angle': 0.08},
            {'type': 'forward', 'distance': 0.99, 'velocity': 0.5}
        ]
        self._send_sequence(sequence, is_primary=True)

    def send_secondary(self):
        """ 发送次序列 """
        sequence = [
            {'type': 'turn', 'angle': -1.59},
            {'type': 'forward', 'distance': 1.2, 'velocity': 0.5}
        ]
        self._send_sequence(sequence)

    def _send_sequence(self, sequence, is_primary=False):
        """ 发送动作序列 """
        goal_msg = MotionSequence.Goal()
        goal_msg.motion_sequence = yaml.dump(sequence)
        
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(
            lambda future: self._goal_response_callback(future, is_primary)
        )

    def _goal_response_callback(self, future, is_primary):
        """ 目标响应回调 """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('目标被拒绝')
            self.primary_done = True
            self.primary_success = False
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(
            lambda result_future: self._result_callback(result_future, is_primary)
        )

    def _result_callback(self, future, is_primary):
        """ 结果回调 """
        try:
            result = future.result().result
            if is_primary:
                self.primary_done = True
                self.primary_success = result.success
                self.get_logger().info('主序列执行完成' if result.success else '主序列执行失败')
        except Exception as e:
            self.get_logger().error(f'结果处理异常: {str(e)}')
            if is_primary:
                self.primary_done = True
                self.primary_success = False

def main(args=None):
    rclpy.init(args=args)
    client = SyncMotionClient()
    
    # Step 1: 执行主序列
    client.get_logger().info("启动主序列...")
    client.send_primary()
    
    # 等待主序列完成（非阻塞式）
    while rclpy.ok() and not client.primary_done:
        rclpy.spin_once(client, timeout_sec=0.1)  # 处理回调
        time.sleep(0.1)  # 降低CPU占用
    
    # Step 2: 根据条件判断是否执行次序列
    if client.primary_success:
        # 这里添加自定义条件判断
        ###############################
        # 示例条件1：固定条件
        #execute_secondary = True
        
        # 示例条件2：基于时间的动态条件
        # current_minute = time.localtime().tm_min
        # execute_secondary = (current_minute % 2 == 0)
        
        # 示例条件3：用户输入
        user_input = input("执行次序列? (y/n): ")
        execute_secondary = (user_input.lower() == 'y')
        ###############################
        
        if execute_secondary:
            client.get_logger().info("满足条件，执行次序列...")
            client.send_secondary()
            
            # 等待次序列完成
            try:
                while rclpy.ok():
                    rclpy.spin_once(client, timeout_sec=0.1)
            except KeyboardInterrupt:
                pass
        else:
            client.get_logger().info("条件未满足，终止程序")
    else:
        client.get_logger().error("主序列失败，终止程序")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
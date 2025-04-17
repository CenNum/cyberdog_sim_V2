# run_motion.py
import rclpy
import asyncio
from rclpy.action import ActionClient
from rclpy.node import Node
from motion_control.action_motion_client import MotionClient
from cyberdog_interfaces.action import MotionSequence
from msgs_lane.srv import Qrcode

class MotionController(Node):
    def __init__(self):
        super().__init__('motion_controller')
        self.sequences = {}          # 存储所有可用序列
        self.current_sequence = None # 当前执行序列ID
        self.is_executing = False    # 执行状态标志
        
        # 创建底层动作客户端
        self._action_client = ActionClient(
            self, 
            MotionSequence, 
            'execute_motion_sequence'
        )

    def add_sequence(self, seq_id, sequence):
        """注册新动作序列"""
        if seq_id in self.sequences:
            self.get_logger().warning(f"序列ID {seq_id} 已存在，将被覆盖")
        self.sequences[seq_id] = sequence
        self.get_logger().info(f"已添加序列 {seq_id}")

    def set_current_sequence(self, seq_id):
        """设置当前要执行的序列"""
        if seq_id not in self.sequences:
            self.get_logger().error(f"无效序列ID {seq_id}")
            return False
            
        self.current_sequence = seq_id
        self.get_logger().info(f"当前序列已设置为 {seq_id}")
        return True

    async def execute_async(self, seq_id=None):
        """异步执行指定序列"""
        if self.is_executing:
            self.get_logger().warning("已有动作正在执行")
            return False

        target_id = seq_id or self.current_sequence
        if not target_id:
            self.get_logger().error("未指定有效序列")
            return False

        try:
            self.is_executing = True
            client = MotionClient()
            
            # 发送目标并等待结果
            goal_future = client.send_goal_async(self.sequences[target_id])
            await goal_future
            result_future = goal_future.result().get_result_async()
            await result_future
            
            return result_future.result().success
        except Exception as e:
            self.get_logger().error(f"执行异常: {str(e)}")
            return False
        finally:
            self.is_executing = False
            client.destroy_node()

    def execute_blocking(self, seq_id=None):
        """阻塞式执行（用于同步逻辑）"""
        rclpy.init()
        try:
            success = asyncio.get_event_loop().run_until_complete(
                self.execute_async(seq_id))
            return success
        finally:
            rclpy.shutdown()

    def emergency_stop(self):
        """紧急停止当前动作"""
        if self.is_executing:
            self._action_client.cancel_all_goals()
            self.get_logger().warn("已发送停止指令")

# ================= 使用示例 =================


def main(args=None):
    # 定义动作序列库
    SEQUENCE_LIB = {
        "move_forward": [
            {"type": "forward", "distance": 1.0, "velocity": 0.5}
        ],
        "turn_left": [
            {"type": "turn", "angle": 1.57}  # 90度左转
        ],
        "complex_move": [
            {"type": "wait", "duration": 2.0},
            {"type": "forward", "distance": 0.5, "velocity": 0.3},
            {"type": "turn", "angle": -0.785}  # 45度右转
        ]
    }

    # 初始化控制器
    rclpy.init()
    controller = MotionController()
    
    # 注册所有动作序列
    for seq_id, sequence in SEQUENCE_LIB.items():
        controller.add_sequence(seq_id, sequence)

    qr_client = controller.create_client(Qrcode, 'Qrcode')

    # # 等待服务可用
    # if not controller.qr_client.wait_for_service(timeout_sec=1.0):
    #     controller.get_logger().error('QR码服务不可用')
    
    #     controller.get_logger().info('等待扫描QR码...')
    #     request = Qrcode.Request()
    #     future = controller.qr_client.call_async(request)

    #     # 等待服务响应
    #     rclpy.spin_until_future_complete(controller, future)
    #     response = future.result()

    #     if response.success:
    #         controller.get_logger().info(f'检测到QR码: {response.content}')
                
    #         if response.content == 'A-1':
    #             # 左转序列
    #             controller.set_current_sequence("move_forward")        

    
    #         elif response.content == 'A-2':
    #         # 右转序列
    #             controller.set_current_sequence("turn_left")        
    #         else:
    #             controller.get_logger().warn(f'未知的QR码内容: {response.content}')

                
    #     else:
    #         controller.get_logger().error('QR码扫描失败')



    try:
        # 示例1：按条件执行
        current_state = "obstacle_detected"
        if current_state == "clear_path":
            controller.set_current_sequence("move_forward")
            controller.execute_blocking()
        elif current_state == "obstacle_detected":
            controller.execute_blocking("turn_left")
            controller.execute_blocking("complex_move")

        # 示例2：动态组合序列
        # route = ["move_forward", "turn_left", "complex_move"]
        # for step in route:
        #     if controller.execute_blocking(step):
        #         print(f"成功完成 {step}")
        #     else:
        #         print(f"执行失败于 {step}")
        #         break

    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
    
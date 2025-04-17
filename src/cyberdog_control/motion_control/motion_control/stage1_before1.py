import yaml
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter

class MotionController(Node):
    def __init__(self):
        super().__init__('motion_controller')
        self.client = self.create_client(SetParameters, '/cyberdog_walk_server/set_parameters')
        self._wait_for_service()
        
        # 预定义运动序列
        self.sequence_presets = {
            'step1': self._create_preset_sequence(1),
            'step2': self._create_preset_sequence(2),
            'backward': """
            - type: forward
              distance: -1.0
              velocity: 0.5
            """
        }

    def _wait_for_service(self):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('等待参数服务...')

    def _create_preset_sequence(self, step: int) -> str:
        """创建预设运动序列"""
        if step == 1:
            return """
            - type: wait
              duration: 3.0
            - type: turn
              angle: 0.08
            - type: forward
              distance: 0.95
              velocity: 0.5
            - type: turn
              angle: -1.59
            - type: forward
              distance: 1.3
              velocity: 0.5
            """
        elif step == 2:
            return """
            - type: forward
              distance: 0.4
              velocity: 0.5
            - type: turn
              angle: 0.60
            - type: forward
              distance: 1.0
              velocity: 0.5
            - type: turn
              angle: -0.60
            - type: strafe
              direction: left
              distance: 0.38
              velocity: 0.3
            - type: forward
              distance: -1.2
              velocity: 0.5
            """
        else:
            return ""

    def set_motion_sequence(self, sequence: str, is_preset: bool = True):
        """
        设置运动序列主方法
        :param sequence: 预设序列名称或YAML字符串
        :param is_preset: 是否为预设序列
        """
        try:
            # 获取YAML内容
            if is_preset:
                if sequence not in self.sequence_presets:
                    raise ValueError(f"未知预设序列: {sequence}")
                yaml_str = self.sequence_presets[sequence]
            else:
                yaml_str = sequence

            # 验证并发送
            validated = self._validate_sequence(yaml_str)
            self._send_parameter(validated)
            return True
        except ValueError as e:
            self.get_logger().error(f"参数错误: {str(e)}")
            return False
        except Exception as e:
            self.get_logger().error(f"未知错误: {str(e)}")
            return False

    def _validate_sequence(self, yaml_str: str) -> str:
        """验证YAML格式"""
        try:
            data = yaml.safe_load(yaml_str)
            if not isinstance(data, list):
                raise ValueError("运动序列必须是动作列表")
            return yaml_str
        except yaml.YAMLError as e:
            raise ValueError(f"YAML语法错误: {str(e)}")

    def _send_parameter(self, valid_yaml: str):
        """发送参数到服务端"""
        param = Parameter(
            name='motion_sequence',
            type_=Parameter.Type.STRING,
            value=valid_yaml
        ).to_parameter_msg()

        request = SetParameters.Request()
        request.parameters = [param]
        
        self.client.call_async(request).add_done_callback(
            lambda future: self.get_logger().info("参数已发送") if future.result().results[0].successful 
            else self.get_logger().error("发送失败")
        )

def main(args=None):
    rclpy.init(args=args)
    controller = MotionController()

    # 使用方式1：调用预设序列
    controller.set_motion_sequence('step1')

    # 使用方式2：发送自定义序列
    # custom_seq = """
    # - type: turn
    #   angle: 1.57
    # - type: wait
    #   duration: 2.0
    # """
    # controller.set_motion_sequence(custom_seq, is_preset=False)

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
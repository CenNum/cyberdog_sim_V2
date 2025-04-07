完整功能示例
ros2 param set /dog_control motion_sequence "
- type: wait
  duration: 0.2
- type: forward
  distance: 1.0
  velocity: 0.5
- type: turn
  angle: -1.57
- type: forward
  distance: 1.2
  velocity: 0.5
- type: strafe
  direction: left
  distance: 0.5
  velocity: 0.3
"


出库到二维码
ros2 param set /dog_control motion_sequence "
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
"

二维码到左库
ros2 param set /dog_control motion_sequence "
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
"


ros2 param set /dog_control motion_sequence "
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
"
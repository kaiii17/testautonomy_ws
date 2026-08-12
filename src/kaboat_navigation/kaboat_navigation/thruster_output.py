import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial


class ThrusterOutput(Node):
    """
    cmd_final(Twist) → 좌우 차동 PWM → 좌현/우현 아두이노 각각 전송.
    좌현(ACM0): 왼쪽 값 하나  "1700\\n"
    우현(ACM1): 오른쪽 값 하나 "1700\\n"
    각 아두이노는 ESC 1개씩 제어 (좌현 9번, 우현 11번).
    baud 115200 (아두이노 스케치와 일치).
    """

    NEUTRAL = 1500
    SCALE = 400        # 전후진 강도 (linear.x=1.0 → ±400)
    TURN = 200         # 회전 강도 (angular.z=1.0 → ±200)
    PWM_MIN = 1100
    PWM_MAX = 1900

    PORT_LEFT = '/dev/ttyACM0'    # 좌현
    PORT_RIGHT = '/dev/ttyACM1'   # 우현
    BAUD = 115200

    def __init__(self):
        super().__init__('thruster_output')

        self.create_subscription(Twist, 'cmd_final', self.cmd_cb, 10)

        self.ard_left = self.connect(self.PORT_LEFT, '좌현')
        self.ard_right = self.connect(self.PORT_RIGHT, '우현')

        if self.ard_left and self.ard_right:
            self.get_logger().info('★ 좌현+우현 모두 연결 완료 ★')
        elif self.ard_left:
            self.get_logger().warn('우현 연결 안됨 - 좌현만 작동')
        elif self.ard_right:
            self.get_logger().warn('좌현 연결 안됨 - 우현만 작동')
        else:
            self.get_logger().error('좌현+우현 모두 연결 실패!')

        self.last_cmd_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.safety_check)

        self.get_logger().info('스러스터 출력 노드 시작')

    def connect(self, port, name):
        try:
            ard = serial.Serial(port, self.BAUD, timeout=1, write_timeout=1)
            self.get_logger().info(f'✓ {name} 연결 완료 ({port}, {self.BAUD})')
            return ard
        except Exception as e:
            self.get_logger().warn(f'✗ {name} 연결 실패 ({port}): {e}')
            return None

    def cmd_cb(self, msg):
        self.last_cmd_time = self.get_clock().now()

        linear = msg.linear.x
        angular = msg.angular.z

        left = self.NEUTRAL + linear * self.SCALE - angular * self.TURN
        right = self.NEUTRAL + linear * self.SCALE + angular * self.TURN

        left = self.constrain(left)
        right = self.constrain(right)

        self.send(left, right)

    def constrain(self, pwm):
        return int(max(self.PWM_MIN, min(self.PWM_MAX, pwm)))

    def send(self, left, right):
        # 좌현엔 왼쪽 값, 우현엔 오른쪽 값 (각각 숫자 하나)
        if self.ard_left is not None:
            try:
                self.ard_left.write(f'{left}\n'.encode())
                self.ard_left.reset_input_buffer()   # 아두이노 디버그 응답 버림
            except Exception as e:
                self.get_logger().warn(f'좌현 전송 실패: {e}')
        if self.ard_right is not None:
            try:
                self.ard_right.write(f'{right}\n'.encode())
                self.ard_right.reset_input_buffer()
            except Exception as e:
                self.get_logger().warn(f'우현 전송 실패: {e}')

    def safety_check(self):
        # 0.5초 이상 명령 없으면 정지
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > 0.5:
            self.send(self.NEUTRAL, self.NEUTRAL)


def main(args=None):
    rclpy.init(args=args)
    node = ThrusterOutput()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 양쪽 중립
        for ard in (node.ard_left, node.ard_right):
            if ard is not None:
                try:
                    ard.write(b'1500\n')
                    ard.close()
                except Exception:
                    pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

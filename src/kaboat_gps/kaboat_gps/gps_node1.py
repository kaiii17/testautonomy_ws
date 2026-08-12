import rclpy
from rclpy.node import Node

from smc_3000_msgs.msg import DrpvaA, ImuattA
from std_msgs.msg import String


class GpsNode1(Node):
    def __init__(self):
        super().__init__('gps_node1')

        self.drpva_sub = self.create_subscription(
            DrpvaA, 'smc3000/drpva', self.drpva_callback, 10)
        self.imu_sub = self.create_subscription(
            ImuattA, 'smc3000/imuatta', self.imu_callback, 10)

        self.nav_pub = self.create_publisher(String, 'kaboat/gps_nav', 10)

        self.imu_heading = None
        self.imu_roll = None
        self.imu_pitch = None
        self.was_sol = False

        self.get_logger().info('gps_node1 시작 - SOL_COMPUTED일 때만 navigation에 발행')

    def imu_callback(self, msg):
        self.imu_heading = msg.heading
        self.imu_roll = msg.roll
        self.imu_pitch = msg.pitch

    def drpva_callback(self, msg):
        is_sol = (msg.sol_status == 'SOL_COMPUTED')

        if is_sol and not self.was_sol:
            self.get_logger().info('★ SOL_COMPUTED 진입! 이제 heading 주행 가능 ★')
        if not is_sol and self.was_sol:
            self.get_logger().warn('SOL_COMPUTED 벗어남 (다시 정렬 중)')
        self.was_sol = is_sol

        if not is_sol:
            return

        imu_h = self.imu_heading if self.imu_heading is not None else 0.0
        out = String()
        out.data = (
            f'lat={msg.lat:.8f},lon={msg.lon:.8f},hgt={msg.hgt:.2f},'
            f'drpva_heading={msg.heading:.3f},'
            f'imu_heading={imu_h:.3f},'
            f've={msg.ve:.3f},vn={msg.vn:.3f}'
        )
        self.nav_pub.publish(out)

        self.get_logger().info(
            f'발행 -> 위치({msg.lat:.6f},{msg.lon:.6f}) '
            f'DRPVA heading:{msg.heading:.1f}도 IMU heading:{imu_h:.1f}도'
        )


def main(args=None):
    rclpy.init(args=args)
    node = GpsNode1()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

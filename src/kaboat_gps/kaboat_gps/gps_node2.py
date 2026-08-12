import time
import rclpy
from rclpy.node import Node

from smc_3000_msgs.msg import DrpvaA, ImuattA, Nmea


class GpsNode2(Node):
    def __init__(self):
        super().__init__("gps_node2")

        self.drpva = None
        self.drpva_time = None
        self.was_sol = False

        self.imu = None
        self.imu_time = None

        self.gngga = None
        self.gngga_time = None

        self.create_subscription(DrpvaA, 'smc3000/drpva', self.drpva_callback, 10)
        self.create_subscription(ImuattA, 'smc3000/imuatta', self.imu_callback, 10)
        self.create_subscription(Nmea, 'smc3000/gngga', self.gngga_callback, 10)

        self.timer = self.create_timer(0.5, self.print_status)
        self.get_logger().info("gps_node2 모니터 시작 (DRPVAA/IMUATTA/GNGGA)")

    def drpva_callback(self, msg):
        self.drpva = msg
        self.drpva_time = time.time()

        is_sol = (msg.sol_status == 'SOL_COMPUTED')
        if is_sol and not self.was_sol:
            self.get_logger().info('★★★ SOL_COMPUTED 진입! 값 신뢰 가능 ★★★')
        if not is_sol and self.was_sol:
            self.get_logger().warn('SOL_COMPUTED 벗어남 -> 다시 정렬 중')
        self.was_sol = is_sol

    def imu_callback(self, msg):
        self.imu = msg
        self.imu_time = time.time()

    def gngga_callback(self, msg):
        self.gngga = msg
        self.gngga_time = time.time()

    def age_text(self, last_time):
        if last_time is None:
            return "No data"
        age = time.time() - last_time
        return "OK" if age < 1.0 else f"{age:.1f} sec ago"

    def print_status(self):
        print("\033c", end="")
        print("========== SMC3000 DATA MONITOR ==========\n")

        print("[DRPVAA] 융합 위치 + Heading + SOL 상태")
        print(f"Receive    : {self.age_text(self.drpva_time)}")
        if self.drpva is not None:
            sol = self.drpva.sol_status
            if sol == 'SOL_COMPUTED':
                print(f"SOL Status : [ {sol} ]  <- 유효! 값 신뢰 가능")
            else:
                print(f"SOL Status : [ {sol} ]  <- 정렬 중, 값 아직 못 믿음")
            print(f"Latitude   : {self.drpva.lat:.8f}")
            print(f"Longitude  : {self.drpva.lon:.8f}")
            print(f"Height     : {self.drpva.hgt:.3f} m")
            print(f"Heading    : {self.drpva.heading:.3f} deg")
            print(f"Pitch      : {self.drpva.pitch:.3f} deg")
            print(f"Roll       : {self.drpva.roll:.3f} deg")
        else:
            print("SOL Status : No DRPVAA data")

        print("\n------------------------------------------\n")

        print("[IMUATTA] IMU 자세 (정렬 전에도 나옴)")
        print(f"Receive    : {self.age_text(self.imu_time)}")
        if self.imu is not None:
            print(f"Heading    : {self.imu.heading:.3f} deg")
            print(f"Roll       : {self.imu.roll:.3f} deg")
            print(f"Pitch      : {self.imu.pitch:.3f} deg")
        else:
            print("Heading    : No IMU data")

        print("\n------------------------------------------\n")

        print("[GNGGA] GPS 안테나 위치 / 위성")
        print(f"Receive    : {self.age_text(self.gngga_time)}")
        if self.gngga is not None:
            print(f"Latitude   : {self.gngga.latitude:.6f}")
            print(f"Longitude  : {self.gngga.longitude:.6f}")
            print(f"Fix Quality: {self.gngga.fix_quality}")
            print(f"Satellites : {self.gngga.num_satellites}")
        else:
            print("Latitude   : No GNGGA data")

        print("\n==========================================")
        print("Ctrl + C 로 종료")


def main(args=None):
    rclpy.init(args=args)
    node = GpsNode2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

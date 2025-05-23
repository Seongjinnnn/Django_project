import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Int32
from tier4_debug_msgs.msg import Float64Stamped
from tf2_msgs.msg import TFMessage
from mgrs import MGRS
from pathlib import Path
import json
import math
import time

CHARGER_X = 11622.964
CHARGER_Y = 90666.18
CHARGER_RADIUS = 5.0

class Listener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)
        self.create_subscription(Int32, '/test', self.soc_callback, 10)
        self.create_subscription(Float64Stamped, '/path_distance_calculator/distance', self.eta_callback, 10)

        self.soc = None
        self.latest_eta = None
        self.last_tf_data = None
        self.last_tf_time = time.time()
        self.last_save_time = 0
        self.mgrs_converter = MGRS()

        self.create_timer(1.0, self.check_tf_timeout)

    def tf_callback(self, msg):
        for tf in msg.transforms:
            if tf.child_frame_id == "base_link":
                t = tf.transform.translation
                self.last_tf_data = (t.x, t.y)
                self.last_tf_time = time.time()

    def soc_callback(self, msg):
        self.soc = msg.data

    def eta_callback(self, msg):
        distance = msg.data
        speed = 3.611  # 13kph
        if distance <= 0:
            self.latest_eta = None
            return
        eta = distance / speed
        self.latest_eta = round(eta)
        self.get_logger().info(f"ETA: {eta:.1f}sec")

    def check_tf_timeout(self):
        if time.time() - self.last_tf_time > 3.0:
            vehicle_info = {
                "lat": 0.0,
                "lng": 0.0,
                "title": "차량 위치 없음",
                "soc": self.soc,
                "status": "위치 없음"
            }
            self.save_vehicle_info(vehicle_info)
        else:
            self.publish_vehicle_info()

    def publish_vehicle_info(self):
        if not self.last_tf_data:
            return
        x, y = self.last_tf_data
        try:
            x_round = int(round(x))
            y_round = int(round(y))
            mgrs_str = f"52SCD{x_round:05d}{y_round:05d}"
            lat, lon = self.mgrs_converter.toLatLon(mgrs_str)

            status = self.determine_status(x, y)

            vehicle_info = {
                "lat": round(lat, 8),
                "lng": round(lon, 8),
                "title": "차량 위치",
                "soc": self.soc,
                "eta": self.latest_eta,
                "status": status
            }
            self.save_vehicle_info(vehicle_info)

        except Exception as e:
            self.get_logger().error(f"[error] {e}")

    def determine_status(self, x, y):
        # 거리 계산
        dx = x - CHARGER_X
        dy = y - CHARGER_Y
        dist = math.sqrt(dx*dx + dy*dy)

        if dist <= CHARGER_RADIUS:
            return "충전중"
        elif self.soc is not None:
            return "운행정지" if self.soc <= 30 else "운행중"
        else:
            return "알수없음"

    def save_vehicle_info(self, vehicle_info):
        now = time.time()
        if now - self.last_save_time < 0.1:
            return
        self.last_save_time = now

        path = Path("vehicle_info.json")
        with path.open("w") as f:
            json.dump({"vehicle": [vehicle_info]}, f, indent=2)

def main():
    rclpy.init()
    node = Listener()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

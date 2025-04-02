import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from autoware_adapi_v1_msgs.msg import MotionState
from pathlib import Path
import json
import time
from mgrs import MGRS

class Listener(Node):
    def __init__(self):
        super().__init__('tf_listener')

        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.listener_callback,
            10
        )

        self.motion_state = None
        self.mgrs_converter = MGRS()
        self.last_tf_time = time.time()
        self.last_save_time = 0  # 🔸 저장 시간 추적

        self.create_subscription(
            MotionState,
            '/api/motion/state',
            self.motion_callback,
            10
        )

        self.create_timer(1.0, self.check_tf_timeout)

    def motion_callback(self, msg):
        self.motion_state = msg.state
        print(f"[🟢 MotionState 수신] 상태 코드: {self.motion_state}")

    def listener_callback(self, msg):
        for tf in msg.transforms:
            if tf.child_frame_id == "base_link":
                t = tf.transform.translation
                self.last_tf_time = time.time()

                try:
                    # Autoware map 좌표는 MGRS 기반
                    x = int(round(t.x))
                    y = int(round(t.y))
                    mgrs_str = f"52SCD{x:05d}{y:05d}"
                    lat, lon = self.mgrs_converter.toLatLon(mgrs_str)

                    marker = {
                        "lat": round(lat, 8),
                        "lng": round(lon, 8),
                        "title": "차량 위치"
                    }

                    self.save_marker(marker)
                    print(f"[✅ TF 변환] x={t.x:.2f}, y={t.y:.2f} → {mgrs_str} → 위도={lat:.8f}, 경도={lon:.8f}")

                except Exception as e:
                    print(f"[⚠️ 변환 오류] {e}")

    def check_tf_timeout(self):
        if time.time() - self.last_tf_time > 3.0:
            marker = {
                "lat": 0.0,
                "lng": 0.0,
                "title": "차량 위치 없음",
                "status": "알수없음"
            }
            self.save_marker(marker)
            print("[⏰ TF 타임아웃] 3초 이상 위치 없음 → 상태: 알수없음")

    def save_marker(self, marker):
        now = time.time()
        if now - self.last_save_time < 0.1:
            return  # 🔸 저장 제한 (0.2초 간격)
        self.last_save_time = now

        # 상태 추가
        if "status" not in marker:
            if self.motion_state == 1:
                marker["status"] = "정지"
            elif self.motion_state == 3:
                marker["status"] = "운행중"
            elif self.motion_state is None:
                marker["status"] = "알수없음"
            else:
                marker["status"] = "알수없음"

        print(f"[💾 상태 저장] {marker['status']}")
        path = Path("marker_data.json")
        with path.open("w") as f:
            json.dump({"markers": [marker]}, f, indent=2)


def main():
    rclpy.init()
    node = Listener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

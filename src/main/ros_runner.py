import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from autoware_adapi_v1_msgs.msg import MotionState
from pathlib import Path
import json
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

        # ⏱️ 상태 초기화: 아직 수신 전
        self.motion_state = None
        self.mgrs_converter = MGRS()

        self.create_subscription(
            MotionState,
            '/api/motion/state',
            self.motion_callback,
            10
        )

    def motion_callback(self, msg):
        self.motion_state = msg.state
        print(f"[🟢 MotionState 수신] 상태: {self.motion_state}")

    def listener_callback(self, msg):
        for tf in msg.transforms:
            if tf.child_frame_id == "base_link":
                t = tf.transform.translation

                try:
                    # MGRS 변환용: x=Easting, y=Northing
                    easting = int(round(t.x))
                    northing = int(round(t.y))

                    mgrs_str = f"52SCD{easting:05d}{northing:05d}"
                    lat, lon = self.mgrs_converter.toLatLon(mgrs_str)

                    marker = {
                        "lat": round(lat, 8),
                        "lng": round(lon, 8),
                        "title": "차량 위치"
                    }

                    self.save_marker(marker)
                    print(f"[✅ TF] x={t.x:.2f}, y={t.y:.2f} → MGRS {mgrs_str} → lat={lat:.8f}, lon={lon:.8f}")

                except Exception as e:
                    print(f"[⚠️ 변환 오류] {e}")

    def save_marker(self, marker):
        # motion_state에 따라 상태 문자열 결정
        if self.motion_state == 1:
            marker["status"] = "정지"
        elif self.motion_state == 3:
            marker["status"] = "운행중"
        elif self.motion_state is None:
            marker["status"] = "대기중"  # 아직 상태 안 들어옴
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

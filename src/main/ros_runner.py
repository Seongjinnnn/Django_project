import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from pathlib import Path
import json
from mgrs import MGRS

class TFListener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.listener_callback,
            10
        )
        self.mgrs_converter = MGRS()

    def listener_callback(self, msg):
        for tf in msg.transforms:
            if tf.child_frame_id == "base_link":
                t = tf.transform.translation

                # ✅ 이번엔 x = 동쪽 (easting), y = 북쪽 (northing)
                try:
                    easting = int(round(t.x))    # 동쪽
                    northing = int(round(t.y))   # 북쪽

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
        path = Path("marker_data.json")
        with path.open("w") as f:
            json.dump({"markers": [marker]}, f, indent=2)

def main():
    rclpy.init()
    node = TFListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

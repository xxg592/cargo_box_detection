
import json
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException


class CargoPositionFusionNode(Node):
    def __init__(self):
        super().__init__("cargo_position_fusion_node")
        self.cloud = None
        self.cloud_received = 0.0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            PointCloud2, "/cargo_robot/depth_camera/points",
            self.cloud_callback, 10
        )
        self.create_subscription(
            String, "/cargo_box/detections", self.detection_callback, 10
        )
        self.publisher = self.create_publisher(
            String, "/cargo_box/positions", 10
        )
        self.get_logger().info(
            "Cargo position fusion started; publishing base_link and odom coordinates."
        )

    def cloud_callback(self, msg):
        self.cloud = msg
        self.cloud_received = time.monotonic()

    @staticmethod
    def transform_matrix(transform):
        t = transform.transform.translation
        q = transform.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        rotation = np.array([
            [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]
        ], dtype=np.float64)
        return rotation, np.array([t.x, t.y, t.z], dtype=np.float64)

    @staticmethod
    def cloud_array(msg):
        fields = {f.name: f for f in msg.fields}
        for name in ("x", "y", "z"):
            if name not in fields or fields[name].datatype != PointField.FLOAT32:
                raise ValueError(f"Missing or invalid point field: {name}")
        endian = ">" if msg.is_bigendian else "<"
        dtype = np.dtype({
            "names": ["x", "y", "z"],
            "formats": [endian+"f4"] * 3,
            "offsets": [fields["x"].offset, fields["y"].offset, fields["z"].offset],
            "itemsize": msg.point_step
        })
        return np.ndarray(
            (msg.height, msg.width), dtype=dtype, buffer=msg.data,
            strides=(msg.row_step, msg.point_step)
        )

    def get_transform(self, target, source):
        try:
            tf = self.tf_buffer.lookup_transform(target, source, Time())
            return self.transform_matrix(tf)
        except TransformException as error:
            self.get_logger().debug(f"TF unavailable {target} <- {source}: {error}")
            return None

    def detection_callback(self, message):
        try:
            result = json.loads(message.data)
            if not isinstance(result, dict):
                raise ValueError("Detection message must be a JSON object.")
            if self.cloud is None or time.monotonic() - self.cloud_received > 1.0:
                return

            cloud = self.cloud
            points = self.cloud_array(cloud)
            base_tf = self.get_transform("base_link", cloud.header.frame_id)
            odom_tf = self.get_transform("odom", cloud.header.frame_id)
            map_tf = self.get_transform("map", cloud.header.frame_id)
            sx = cloud.width / float(result["image_width"])
            sy = cloud.height / float(result["image_height"])
            objects = []

            for obj in result.get("detections", []):
                x, y, w, h = obj["bbox"]
                x1 = max(0, int((x + 0.25*w) * sx))
                x2 = min(cloud.width, int((x + 0.75*w) * sx))
                y1 = max(0, int((y + 0.25*h) * sy))
                y2 = min(cloud.height, int((y + 0.75*h) * sy))
                if x2 <= x1 or y2 <= y1:
                    continue

                roi = points[y1:y2, x1:x2]
                xyz = np.column_stack((
                    roi["x"].ravel(), roi["y"].ravel(), roi["z"].ravel()
                )).astype(np.float64, copy=False)
                distance = np.linalg.norm(xyz, axis=1)
                xyz = xyz[
                    np.isfinite(xyz).all(axis=1)
                    & (distance > 0.1) & (distance <= 5.0)
                ]
                if len(xyz) < 5:
                    continue

                camera_xyz = np.median(xyz, axis=0)
                item = {
                    "color": str(obj.get("color", "UNKNOWN")),
                    "camera_xyz_m": [round(float(v), 4) for v in camera_xyz],
                    "distance_camera_m": round(float(np.linalg.norm(camera_xyz)), 4),
                    "base_link_xyz_m": None,
                    "distance_base_link_m": None,
                    "odom_xyz_m": None,
                    "distance_odom_m": None,
                    "map_xyz_m": None,
                    "distance_map_m": None
                }

                if base_tf is not None:
                    rot, trans = base_tf
                    base_xyz = rot @ camera_xyz + trans
                    item["base_link_xyz_m"] = [round(float(v), 4) for v in base_xyz]
                    item["distance_base_link_m"] = round(float(np.linalg.norm(base_xyz)), 4)

                if odom_tf is not None:
                    rot, trans = odom_tf
                    odom_xyz = rot @ camera_xyz + trans
                    item["odom_xyz_m"] = [round(float(v), 4) for v in odom_xyz]
                    item["distance_odom_m"] = round(float(np.linalg.norm(odom_xyz)), 4)

                if map_tf is not None:
                    rot, trans = map_tf
                    map_xyz = rot @ camera_xyz + trans
                    item["map_xyz_m"] = [round(float(v), 4) for v in map_xyz]
                    item["distance_map_m"] = round(float(np.linalg.norm(map_xyz)), 4)

                objects.append(item)

            report = {
                "frame_id": "base_link",
                "navigation_frame_id": "map",
                "source_frame_id": cloud.header.frame_id,
                "cloud_age_s": round(time.monotonic() - self.cloud_received, 3),
                "objects": objects
            }
            output = String()
            output.data = json.dumps(report)
            self.publisher.publish(output)
            self.get_logger().info(f"Published cargo positions: {len(objects)}")
        except Exception as error:
            self.get_logger().error(f"Position fusion failed: {error}")


def main(args=None):
    rclpy.init(args=args)
    node = CargoPositionFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

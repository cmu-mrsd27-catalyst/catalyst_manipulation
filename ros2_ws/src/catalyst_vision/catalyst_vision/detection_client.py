"""
Well Plate Detection — ROS2 Client Node
=========================================

ROS2 node that:
1. Provides a ROS2 service /detect_well_plate
2. When called, grabs the latest image from the RealSense camera topic
3. Sends the image to the GPU desktop server via HTTP
4. Returns detections and pick/place status

Install in your ROS2 Docker:
    pip install requests

Parameters:
    server_url: URL of the detection server (e.g. http://100.x.x.x:8000)
    camera_topic: RealSense image topic (default: /camera/camera/color/image_raw)

Run:
    ros2 run well_plate_detection detection_client --ros-args \
        -p server_url:=http://100.x.x.x:8000 \
        -p camera_topic:=/camera/camera/color/image_raw

Call the service:
    ros2 service call /detect_well_plate well_plate_detection/srv/DetectWellPlate
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

import cv2
import json
import requests
import numpy as np
from io import BytesIO


# ──────────────────────────────────────────────
# Custom service definition workaround
# ──────────────────────────────────────────────
# Since creating a custom .srv requires a full package build,
# we use a simpler approach: a topic-based request/response
# and also a std_srvs/Trigger-based service.

from std_srvs.srv import Trigger


class DetectionClient(Node):
    def __init__(self):
        super().__init__("well_plate_detection_client")

        # Parameters
        self.declare_parameter("server_url", "http://100.64.0.1:8000")
        self.declare_parameter("camera_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("timeout", 10.0)

        self.server_url = self.get_parameter("server_url").value
        self.camera_topic = self.get_parameter("camera_topic").value
        self.timeout = self.get_parameter("timeout").value

        self.get_logger().info(f"Detection server: {self.server_url}")
        self.get_logger().info(f"Camera topic: {self.camera_topic}")

        # CV Bridge for image conversion
        self.bridge = CvBridge()

        # Store latest image
        self.latest_image = None
        self.latest_image_time = None

        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            1,  # only keep latest
        )

        # Service: /detect_well_plate
        self.detect_srv = self.create_service(
            Trigger,
            "detect_well_plate",
            self.detect_callback,
        )

        # Publisher: publish detections as JSON
        self.detection_pub = self.create_publisher(
            String,
            "well_plate_detections",
            10,
        )

        self.get_logger().info("Detection client ready. Call /detect_well_plate service.")

    def image_callback(self, msg):
        """Store the latest camera image."""
        self.latest_image = msg
        self.latest_image_time = self.get_clock().now()

    def detect_callback(self, request, response):
        """Handle detection service call."""
        if self.latest_image is None:
            response.success = False
            response.message = json.dumps({
                "error": "No image received yet from camera",
            })
            self.get_logger().warn("No image available from camera topic")
            return response

        # Check image age
        age = (self.get_clock().now() - self.latest_image_time).nanoseconds / 1e9
        if age > 5.0:
            self.get_logger().warn(f"Image is {age:.1f}s old — may be stale")

        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding="bgr8")

            # Encode as JPEG
            success, buffer = cv2.imencode(".jpg", cv_image, [cv2.IMWRITE_JPEG_QUALITY, 90])
            if not success:
                response.success = False
                response.message = json.dumps({"error": "Failed to encode image"})
                return response

            # Send to detection server
            self.get_logger().info("Sending image to detection server...")
            files = {"file": ("image.jpg", BytesIO(buffer.tobytes()), "image/jpeg")}
            resp = requests.post(
                f"{self.server_url}/detect",
                files=files,
                timeout=self.timeout,
            )

            if resp.status_code != 200:
                response.success = False
                response.message = json.dumps({
                    "error": f"Server returned status {resp.status_code}",
                    "detail": resp.text,
                })
                return response

            result = resp.json()

            # Log results
            n_detections = len(result.get("detections", []))
            pick_safe = result.get("pick_place", {}).get("pick_safe", False)
            place_safe = result.get("pick_place", {}).get("place_safe", False)
            inference_ms = result.get("inference_time_ms", 0)

            self.get_logger().info(
                f"Detections: {n_detections} | "
                f"Pick: {'SAFE' if pick_safe else 'UNSAFE'} | "
                f"Place: {'SAFE' if place_safe else 'UNSAFE'} | "
                f"Inference: {inference_ms}ms"
            )

            # Publish detections to topic
            det_msg = String()
            det_msg.data = json.dumps(result)
            self.detection_pub.publish(det_msg)

            # Return via service
            response.success = True
            response.message = json.dumps(result)

        except requests.exceptions.ConnectionError:
            response.success = False
            response.message = json.dumps({
                "error": f"Cannot connect to detection server at {self.server_url}",
            })
            self.get_logger().error(f"Connection failed to {self.server_url}")

        except requests.exceptions.Timeout:
            response.success = False
            response.message = json.dumps({
                "error": f"Detection server timed out after {self.timeout}s",
            })
            self.get_logger().error("Detection server timeout")

        except Exception as e:
            response.success = False
            response.message = json.dumps({"error": str(e)})
            self.get_logger().error(f"Detection failed: {e}")

        return response


def main(args=None):
    rclpy.init(args=args)
    node = DetectionClient()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

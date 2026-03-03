# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from pathlib import Path
import cv2
from cv_bridge import CvBridge
import numpy as np


class DepthSubscriber(Node):

    def __init__(self):
        super().__init__('depth_subscriber')

        # convert ROS Image to OpenCV images
        self.bridge = CvBridge()

        self.out_dir = Path.home() / "depth_images"
        self.out_dir.mkdir(parents=True, exist_ok=True)

        self.count = 0

        self.subscription = self.create_subscription(
            Image,
            "/camera/camera/depth/image_rect_raw",  # topic
            self.listener_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info("Subscribed to /camera/camera/depth/image_rect_raw")
        self.get_logger().info(f"Saving images to {self.out_dir}")

    def listener_callback(self, msg: Image):
        self.count += 1

        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"CvBridge failed: {e}")
            return

        filename = self.out_dir / f"depth_{self.count:06d}.png"

        if depth.dtype == np.uint16:
            ok = cv2.imwrite(str(filename), depth)
        elif depth.dtype == np.float32:
            self.get_logger().error(f"Not uint16 depth images (dtype=float32, encoding={msg.encoding})")
            return
        else:
            self.get_logger().error(f"Unsupported dtype {depth.dtype} encoding={msg.encoding}")
            return

        if ok:
            self.get_logger().info(f"Saved {filename.name}")
        else:
            self.get_logger().error(f"Failed to write {filename}")


def main(args=None):
    rclpy.init(args=args)
    node = DepthSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

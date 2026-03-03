import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from message_filters import Subscriber, ApproximateTimeSynchronizer  # or TimeSynchronizer

from pathlib import Path
import cv2
import numpy as np
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped


class SyncedRGBDepthSubscriber(Node):
    def __init__(self):
        super().__init__("synced_rgb_depth_subscriber")

        self.bridge = CvBridge()

        self.rgb_dir = Path.home() / "rgb_images"
        self.depth_dir = Path.home() / "depth_images"
        
        #self.pose_file = Path.home() / 'vicon_poses.csv'
        #self.get_logger().info(f"Saving Vicon poses -> {self.pose_file}")
        
        self.rgb_dir.mkdir(parents=True, exist_ok=True)
        self.depth_dir.mkdir(parents=True, exist_ok=True)

        self.rgb_sub = Subscriber(
            self,
            Image, 
            "/camera/camera/color/image_raw", 
            qos_profile=qos_profile_sensor_data
        )
        self.depth_sub = Subscriber(
            self, 
            Image, 
            "/camera/camera/depth/image_rect_raw",
            qos_profile=qos_profile_sensor_data
        )
        '''
        self.vicon_sub = Subscriber(
            self,
            PoseStamped,
            "/vicon/objname/pose",
            qos_profile=qos_profile_sensor_data
        )
        '''
        #approx sync since depth & rgb are diff hardware than vicon
        queue_size = 30   #30 que size since not real time
        slop_sec = 0.005  #5ms tollerance
        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], #self.vicon_sub],
            queue_size=queue_size,
            slop=slop_sec,
            allow_headerless=False,
        )

        self.sync.registerCallback(self.synced_callback)  

        self.get_logger().info("Subscribed (synced) to:")
        self.get_logger().info("  /camera/camera/color/image_raw")
        self.get_logger().info("  /camera/camera/depth/image_rect_raw")
        #self.get_logger().info("  /vicon/objname/pose")
        #self.get_logger().info(f"Saving vicon -> {self.pose_file}")
        #self.pose_fh = open(self.pose_file, "w")
        #self.pose_fh.write("pair_id,t_ns,px,py,pz,qx,qy,qz,qw\n")
        self.get_logger().info(f"Saving RGB -> {self.rgb_dir}")
        self.get_logger().info(f"Saving Depth -> {self.depth_dir}")

        self.pair_count = 0

    @staticmethod
    def _stamp_to_ns(msg: Image) -> int:
        #ros timestamp -> nano seconds: sec + nanosec
        return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

    def synced_callback(self, rgb_msg: Image, depth_msg: Image):#, vicon_msg: PoseStamped):
        self.pair_count  += 1

        rgb_t = self._stamp_to_ns(rgb_msg)
        depth_t = self._stamp_to_ns(depth_msg)
        #vicon_t = self._stamp_to_ns(vicon_msg)
        
        dt_ms = abs(rgb_t - depth_t) / 1e6

        #pos = vicon_msg.pose.position
        #ori = vicon_msg.pose.orientation

        # px, py, pz = pos.x, pos.y, pos.z
        # qx, qy, qz, qw = ori.x, ori.y, ori.z, ori.w

        # Convert RGB
        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")

        '''
        try:
        except Exception as e:
            self.get_logger().error(f"RGB CvBridge failed: {e}")
            return
        '''
        # Convert Depth
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        '''
        try:
        except Exception as e:
            self.get_logger().error(f"Depth CvBridge failed: {e}")
            return
        '''
        # convert depth to normalized (metric, 9m)
        depth = depth / (1000 * 6)
        depth = (np.clip(depth, 0, 1) * 255).astype(np.uint8)

        # Save with a shared timestamp so you can pair them later
        # Use the RGB timestamp as the "pair id"
        pair_id = f"{rgb_msg.header.stamp.sec}_{rgb_msg.header.stamp.nanosec:09d}"

        rgb_file = self.rgb_dir / f"rgb_{pair_id}.png"
        depth_file = self.depth_dir / f"depth_{pair_id}.png"
        # print(rgb.shape, depth.shape)
        concat = np.concatenate((rgb, np.repeat(depth[..., np.newaxis], repeats=3, axis=2)), axis=1)
        # print(concat.shape)
        concat = cv2.resize(concat, (640*2, 480))
        cv2.imshow('frame', concat)
        # cv2.imshow('frame2', depth)
        if cv2.waitKey(1) == ord('q'):
            exit()

        ok_rgb = cv2.imwrite(str(rgb_file), rgb)

        # self.pose_fh.write(
        #     f"{pair_id},{vicon_t},"
        #     f"{px:.6f},{py:.6f},{pz:.6f},"
        #     f"{qx:.6f},{qy:.6f},{qz:.6f},{qw:.6f}\n"
        # )
        # self.pose_fh.flush()

        ok_depth = False

        if depth.dtype == np.uint8:
            ok_depth = cv2.imwrite(str(depth_file), depth)

        """
        elif depth.dtype == np.float32:
            self.get_logger().error(
                f"Depth is float32 (encoding={depth_msg.encoding}); not saving as PNG."
            )
            return

        else:
            self.get_logger().error(
                f"Unsupported depth dtype {depth.dtype} (encoding={depth_msg.encoding})"
            )
            return
        else:
            self.get_logger().error("Failed to write one or both images.")
        """
        if ok_rgb and ok_depth:
            self.get_logger().info(
                f"Saved pair {self.pair_count}: {rgb_file.name} + {depth_file.name} (|dt|={dt_ms:.2f} ms)"
            )

def main(args=None):
    rclpy.init(args=args)
    node = SyncedRGBDepthSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    #node.pose_fh.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


#!/usr/bin/env python3

"""
Subscribe to the compressed image topic stream from the simulation environment,
decode it, scale it, and display it in real-time.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import qos_profile_sensor_data
import cv2
import numpy as np
import time

class VideoStreamDecoderNode(Node):
    def __init__(self):
        super().__init__('video_stream_decoder_node')

        self.subscription = self.create_subscription(
            CompressedImage,
            '/gazebo_camera/image_raw/compressed',
            self.listener_callback,
            qos_profile_sensor_data)
        self.subscription

        self.frame_count = 0
        self.fps_t0 = time.monotonic()

        self.get_logger().info('Video stream decoder node has started, waiting for compressed image stream...')

    def listener_callback(self, msg):
        try:

            np_arr = np.frombuffer(msg.data, np.uint8)

            raw_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if raw_image is not None:

                self.frame_count += 1
                now = time.monotonic()
                if now - self.fps_t0 >= 1.0:
                    fps = self.frame_count / (now - self.fps_t0)
                    self.get_logger().info(
                        f'Receiving normally | Resolution: {raw_image.shape[1]}x{raw_image.shape[0]} | FPS: {fps:.1f}'
                    )
                    self.frame_count = 0
                    self.fps_t0 = now

                display_img = cv2.resize(raw_image, (0, 0), fx=0.5, fy=0.5)
                cv2.imshow("Downstream: Restored Raw Image", display_img)
                cv2.waitKey(1)

            else:
                self.get_logger().warning('Image decoding failed: Data packet may be corrupted.')

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = VideoStreamDecoderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down node...')
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

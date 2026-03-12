#!/usr/bin/env python3

"""
Simple camera viewer untuk melihat video dari topic ROS2
Pastikan DISPLAY environment variable sudah di-set untuk GUI
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys


class CameraViewer(Node):
    
    def __init__(self, topic_name='/seano/SEANO001/camera/image'):
        super().__init__('camera_viewer')
        
        self.topic_name = topic_name
        self.bridge = CvBridge()
        self.latest_frame = None
        
        # Subscribe ke camera topic
        self.subscription = self.create_subscription(
            Image,
            self.topic_name,
            self.image_callback,
            10
        )
        
        self.get_logger().info(f'Camera Viewer started')
        self.get_logger().info(f'Subscribing to: {self.topic_name}')
        self.get_logger().info('Press Q to quit')
        
    def image_callback(self, msg):
        """Callback saat menerima image dari topic"""
        try:
            # Convert ROS Image message to OpenCV format
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {str(e)}')
    
    def display_loop(self):
        """Loop untuk menampilkan frame"""
        window_name = f'SEANO Camera Viewer - {self.topic_name}'
        
        try:
            while rclpy.ok():
                # Process ROS callbacks
                rclpy.spin_once(self, timeout_sec=0.01)
                
                # Display frame if available
                if self.latest_frame is not None:
                    cv2.imshow(window_name, self.latest_frame)
                    
                    # Check for key press (Q to quit)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q') or key == ord('Q'):
                        self.get_logger().info('Quit requested')
                        break
                else:
                    # Just wait for frame
                    cv2.waitKey(10)
                    
        except KeyboardInterrupt:
            pass
        except Exception as e:
            self.get_logger().error(f'Error in display loop: {str(e)}')
        finally:
            cv2.destroyAllWindows()
            self.get_logger().info('Camera viewer closed')


def main(args=None):
    rclpy.init(args=args)
    
    # Get topic name from command line argument if provided
    topic_name = '/seano/SEANO001/camera/image'
    if len(sys.argv) > 1:
        topic_name = sys.argv[1]
    
    try:
        viewer = CameraViewer(topic_name)
        viewer.display_loop()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

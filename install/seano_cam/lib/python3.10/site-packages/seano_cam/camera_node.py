#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import glob
import subprocess
import cv2
import numpy as np


class CameraNode(Node):
    """
    Node untuk mendeteksi dan mengelola kamera USB.
    Future: Akan ditambahkan kemampuan AI untuk pemrosesan gambar.
    """

    def __init__(self):
        super().__init__('camera_node')
        
        # Declare parameters
        self.declare_parameter('vehicle.id', 'UNKNOWN')
        self.declare_parameter('camera.check_interval', 5.0)  # Check every 5 seconds
        self.declare_parameter('camera.device', '/dev/video0')  # Default camera device
        self.declare_parameter('camera.enable_display', True)  # Show video window
        self.declare_parameter('camera.enable_publish', True)  # Publish to ROS topic
        self.declare_parameter('camera.fps', 30)  # Frame rate
        
        # Get parameters
        self.vehicle_id = self.get_parameter('vehicle.id').value
        self.check_interval = self.get_parameter('camera.check_interval').value
        self.camera_device = self.get_parameter('camera.device').value
        self.enable_display = self.get_parameter('camera.enable_display').value
        self.enable_publish = self.get_parameter('camera.enable_publish').value
        self.fps = self.get_parameter('camera.fps').value
        
        # Publisher untuk status kamera
        self.camera_status_pub = self.create_publisher(
            String, 
            f'/seano/{self.vehicle_id}/camera/status', 
            10
        )
        
        # Publisher untuk image
        if self.enable_publish:
            self.image_pub = self.create_publisher(
                Image,
                f'/seano/{self.vehicle_id}/camera/image',
                10
            )
            self.bridge = CvBridge()
        
        # Timer untuk mengecek kamera secara berkala
        self.check_timer = self.create_timer(self.check_interval, self.check_cameras)
        
        # OpenCV VideoCapture dan timer untuk frame
        self.cap = None
        self.frame_timer = None
        
        self.get_logger().info('Camera Node started')
        self.get_logger().info(f'Vehicle ID: {self.vehicle_id}')
        self.get_logger().info(f'Camera Device: {self.camera_device}')
        self.get_logger().info(f'Display Enabled: {self.enable_display}')
        self.get_logger().info(f'Publish Enabled: {self.enable_publish}')
        
        # Deteksi kamera saat startup
        self.detect_initial_cameras()
        
        # Start video capture
        self.start_video_capture()
    
    def detect_initial_cameras(self):
        """Deteksi kamera saat node pertama kali dijalankan"""
        self.get_logger().info('=' * 50)
        self.get_logger().info('Detecting USB cameras...')
        self.get_logger().info('=' * 50)
        
        cameras = self.get_available_cameras()
        
        if cameras:
            self.get_logger().info(f'Found {len(cameras)} camera(s):')
            for cam in cameras:
                self.get_logger().info(f'  - {cam["device"]}: {cam["name"]}')
                if cam.get('resolution'):
                    self.get_logger().info(f'    Resolution: {cam["resolution"]}')
        else:
            self.get_logger().warn('No USB cameras detected!')
        
        self.get_logger().info('=' * 50)
    
    def get_available_cameras(self):
        """
        Mendeteksi semua kamera USB yang tersedia di sistem.
        Returns: List of dictionaries dengan informasi kamera
        """
        cameras = []
        
        # Cek semua /dev/video* devices
        video_devices = glob.glob('/dev/video*')
        
        for device in sorted(video_devices):
            try:
                # Cek apakah device ini adalah capture device (bukan metadata)
                if self.is_capture_device(device):
                    camera_info = {
                        'device': device,
                        'name': self.get_camera_name(device),
                        'resolution': self.get_camera_resolution(device)
                    }
                    cameras.append(camera_info)
            except Exception as e:
                self.get_logger().debug(f'Error checking {device}: {str(e)}')
        
        return cameras
    
    def is_capture_device(self, device):
        """
        Cek apakah device adalah capture device (bukan metadata device)
        """
        try:
            result = subprocess.run(
                ['v4l2-ctl', '--device', device, '--info'],
                capture_output=True,
                text=True,
                timeout=2
            )
            # Device yang punya capability "Video Capture" adalah camera
            return 'Video Capture' in result.stdout
        except Exception:
            # Jika v4l2-ctl tidak tersedia, cek dengan cara sederhana
            return os.path.exists(device)
    
    def get_camera_name(self, device):
        """
        Mendapatkan nama kamera dari device
        """
        try:
            result = subprocess.run(
                ['v4l2-ctl', '--device', device, '--info'],
                capture_output=True,
                text=True,
                timeout=2
            )
            for line in result.stdout.split('\n'):
                if 'Card type' in line:
                    return line.split(':')[1].strip()
        except Exception:
            pass
        
        return 'Unknown Camera'
    
    def get_camera_resolution(self, device):
        """
        Mendapatkan resolusi yang didukung kamera
        """
        try:
            result = subprocess.run(
                ['v4l2-ctl', '--device', device, '--list-formats-ext'],
                capture_output=True,
                text=True,
                timeout=2
            )
            # Ambil resolusi pertama yang ditemukan
            for line in result.stdout.split('\n'):
                if 'Size:' in line:
                    return line.split(':')[1].strip()
        except Exception:
            pass
        
        return None
    
    def check_cameras(self):
        """
        Callback timer untuk mengecek status kamera secara berkala
        """
        cameras = self.get_available_cameras()
        
        # Publish status
        status_msg = String()
        if cameras:
            camera_list = [f"{cam['device']}:{cam['name']}" for cam in cameras]
            status_msg.data = f"CAMERAS_DETECTED:{','.join(camera_list)}"
            self.get_logger().debug(f'Cameras online: {len(cameras)}')
        else:
            status_msg.data = "NO_CAMERAS"
            self.get_logger().warn('No cameras detected')
        
        self.camera_status_pub.publish(status_msg)


    def start_video_capture(self):
        """
        Memulai video capture dari kamera
        """
        try:
            # Extract camera index from device path (e.g., /dev/video0 -> 0)
            if self.camera_device.startswith('/dev/video'):
                camera_index = int(self.camera_device.replace('/dev/video', ''))
            else:
                camera_index = 0
            
            self.get_logger().info(f'Opening camera {self.camera_device} (index: {camera_index})...')
            
            # Open camera
            self.cap = cv2.VideoCapture(camera_index)
            
            if not self.cap.isOpened():
                self.get_logger().error(f'Failed to open camera {self.camera_device}')
                return
            
            # Set resolution (optional)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            
            # Get actual resolution
            width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            
            self.get_logger().info(f'Camera opened successfully! Resolution: {width}x{height}')
            
            # Start timer for frame capture
            frame_interval = 1.0 / self.fps
            self.frame_timer = self.create_timer(frame_interval, self.capture_frame)
            
        except Exception as e:
            self.get_logger().error(f'Error starting video capture: {str(e)}')
    
    def capture_frame(self):
        """
        Callback untuk mengambil dan memproses frame dari kamera
        """
        if self.cap is None or not self.cap.isOpened():
            return
        
        try:
            # Read frame
            ret, frame = self.cap.read()
            
            if not ret:
                self.get_logger().warn('Failed to read frame from camera')
                return
            
            # Add info text to frame
            height, width = frame.shape[:2]
            cv2.putText(frame, f'SEANO CAM - {self.vehicle_id}', 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f'{width}x{height} @ {self.fps}fps', 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            
            # Display frame (only if GUI is available)
            if self.enable_display:
                try:
                    cv2.imshow(f'SEANO Camera - {self.vehicle_id}', frame)
                    cv2.waitKey(1)  # Important: allows window to be displayed
                except Exception as e:
                    # Disable display if GUI is not available (headless, SSH, etc)
                    if 'GTK' in str(e) or 'X11' in str(e) or 'display' in str(e).lower():
                        self.enable_display = False
                        self.get_logger().warn('GUI display not available. Use rqt_image_view to view camera.')
                        self.get_logger().warn(f'Command: ros2 run rqt_image_view rqt_image_view /seano/{self.vehicle_id}/camera/image')
                    else:
                        raise
            
            # Publish frame to ROS topic
            if self.enable_publish:
                try:
                    image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                    self.image_pub.publish(image_msg)
                except Exception as e:
                    self.get_logger().debug(f'Error publishing image: {str(e)}')
                    
        except Exception as e:
            self.get_logger().error(f'Error capturing frame: {str(e)}')
    
    def cleanup(self):
        """
        Cleanup resources
        """
        if self.cap is not None:
            self.cap.release()
        if self.enable_display:
            cv2.destroyAllWindows()
        self.get_logger().info('Camera resources cleaned up')


def main(args=None):
    rclpy.init(args=args)
    
    camera_node = None
    try:
        camera_node = CameraNode()
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in camera_node: {str(e)}')
    finally:
        if camera_node is not None:
            camera_node.cleanup()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

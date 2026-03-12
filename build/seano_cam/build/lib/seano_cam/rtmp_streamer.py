#!/usr/bin/env python3

"""
RTMP Streamer Node untuk streaming video camera ke RTMP server
Supports streaming to services like YouTube Live, Facebook Live, atau custom RTMP server
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import subprocess
import numpy as np
import sys
import signal


class RTMPStreamer(Node):
    
    def __init__(self):
        super().__init__('rtmp_streamer')
        
        # Declare parameters
        self.declare_parameter('vehicle.id', 'UNKNOWN')
        self.declare_parameter('rtmp.url', 'rtmp://72.61.141.126:1935/live/usv-seano')
        self.declare_parameter('rtmp.fps', 30)
        self.declare_parameter('rtmp.width', 1280)
        self.declare_parameter('rtmp.height', 720)
        self.declare_parameter('rtmp.bitrate', '2500k')
        self.declare_parameter('rtmp.preset', 'ultrafast')  # ultrafast, fast, medium, slow
        self.declare_parameter('camera.topic', '')  # Auto-generate if empty
        self.declare_parameter('overlay.enable', False)  # Enable/disable overlay text
        
        # Get parameters
        self.vehicle_id = self.get_parameter('vehicle.id').value
        self.rtmp_url = self.get_parameter('rtmp.url').value
        self.fps = self.get_parameter('rtmp.fps').value
        self.width = self.get_parameter('rtmp.width').value
        self.height = self.get_parameter('rtmp.height').value
        self.bitrate = self.get_parameter('rtmp.bitrate').value
        self.preset = self.get_parameter('rtmp.preset').value
        self.overlay_enable = self.get_parameter('overlay.enable').value
        
        camera_topic = self.get_parameter('camera.topic').value
        if not camera_topic:
            camera_topic = f'/seano/{self.vehicle_id}/camera/image'
        self.camera_topic = camera_topic
        
        self.bridge = CvBridge()
        self.ffmpeg_process = None
        self.frame_count = 0
        
        # Log configuration
        self.get_logger().info('=' * 60)
        self.get_logger().info('RTMP Streamer Node Started')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Vehicle ID: {self.vehicle_id}')
        self.get_logger().info(f'RTMP URL: {self.rtmp_url}')
        self.get_logger().info(f'Camera Topic: {self.camera_topic}')
        self.get_logger().info(f'Output Resolution: {self.width}x{self.height}')
        self.get_logger().info(f'FPS: {self.fps}')
        self.get_logger().info(f'Bitrate: {self.bitrate}')
        self.get_logger().info(f'Preset: {self.preset}')
        self.get_logger().info('=' * 60)
        
        # Start ffmpeg process
        self.start_ffmpeg()
        
        # Subscribe to camera topic
        self.subscription = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10
        )
        
        self.get_logger().info('Waiting for camera frames...')
        
    def start_ffmpeg(self):
        """Start ffmpeg process for RTMP streaming"""
        try:
            # FFmpeg command for RTMP streaming (low latency optimized)
            command = [
                'ffmpeg',
                '-re',  # Read input at native frame rate
                '-f', 'rawvideo',
                '-vcodec', 'rawvideo',
                '-pix_fmt', 'bgr24',
                '-s', f'{self.width}x{self.height}',
                '-r', str(self.fps),
                '-i', '-',
                '-c:v', 'libx264',
                '-pix_fmt', 'yuv420p',
                '-preset', 'ultrafast',
                '-tune', 'zerolatency',
                '-b:v', self.bitrate,
                '-maxrate', self.bitrate,
                '-bufsize', self.bitrate,  # Small buffer = low latency
                '-g', '15',  # Keyframe every 0.5 sec for responsive playback
                '-sc_threshold', '0',
                '-x264opts', 'no-scenecut',
                '-f', 'flv',
                self.rtmp_url
            ]
            
            self.get_logger().info('Starting ffmpeg process...')
            self.get_logger().debug(f"Command: {' '.join(command)}")
            
            # Start ffmpeg process
            self.ffmpeg_process = subprocess.Popen(
                command,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=self.width * self.height * 3 * 10  # Buffer for 10 frames
            )
            
            self.get_logger().info('✓ FFmpeg started successfully')
            self.get_logger().info(f'✓ Streaming to: {self.rtmp_url}')
            
            # Check if FFmpeg started properly
            import time
            time.sleep(0.5)
            if self.ffmpeg_process.poll() is not None:
                stderr_output = self.ffmpeg_process.stderr.read().decode('utf-8', errors='ignore')
                self.get_logger().error(f'FFmpeg failed to start! Error:\n{stderr_output}')
                raise Exception(f'FFmpeg startup failed: {stderr_output}')
            
        except FileNotFoundError:
            self.get_logger().error('FFmpeg not found! Install with: sudo apt-get install ffmpeg')
            raise
        except Exception as e:
            self.get_logger().error(f'Failed to start ffmpeg: {str(e)}')
            raise
    
    def image_callback(self, msg):
        """Callback when receiving image from camera topic"""
        if self.ffmpeg_process is None or self.ffmpeg_process.poll() is not None:
            # FFmpeg died, log error
            if self.ffmpeg_process is not None:
                stderr_output = self.ffmpeg_process.stderr.read().decode('utf-8', errors='ignore')
                if stderr_output:
                    self.get_logger().error(f'FFmpeg stderr:\n{stderr_output[-500:]}')  # Last 500 chars
            
            self.get_logger().error('FFmpeg process died! Attempting restart...')
            self.start_ffmpeg()
            return
        
        try:
            # Convert ROS Image to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Check frame is valid
            if frame is None or frame.size == 0:
                self.get_logger().warn('Received invalid frame, skipping')
                return
            
            # Resize frame to target resolution
            if frame.shape[1] != self.width or frame.shape[0] != self.height:
                frame = cv2.resize(frame, (self.width, self.height), interpolation=cv2.INTER_LINEAR)
            
            # Add overlay info (optional)
            self.add_overlay(frame)
            
            # Write frame to ffmpeg stdin
            try:
                self.ffmpeg_process.stdin.write(frame.tobytes())
                self.ffmpeg_process.stdin.flush()  # Flush buffer immediately
                self.frame_count += 1
                
                # Log progress every 500 frames (less frequent to reduce lag)
                if self.frame_count % 500 == 0:
                    self.get_logger().info(f'Streamed {self.frame_count} frames', throttle_duration_sec=5.0)
                    
            except BrokenPipeError:
                self.get_logger().error('FFmpeg pipe broken! Stream may have ended.')
                self.cleanup()
                
        except Exception as e:
            self.get_logger().error(f'Error processing frame: {str(e)}')
    
    def add_overlay(self, frame):
        """Add overlay information to frame"""
        # Skip overlay if disabled
        if not self.overlay_enable:
            return
        
        # Add live indicator
        cv2.circle(frame, (30, 30), 10, (0, 0, 255), -1)
        cv2.putText(frame, 'LIVE', (50, 40), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # Add vehicle ID
        cv2.putText(frame, f'{self.vehicle_id}', (50, 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Add frame count
        cv2.putText(frame, f'Frame: {self.frame_count}', (self.width - 200, 40),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    def cleanup(self):
        """Cleanup resources"""
        self.get_logger().info('Cleaning up RTMP streamer...')
        
        if self.ffmpeg_process is not None:
            try:
                self.ffmpeg_process.stdin.close()
                self.ffmpeg_process.wait(timeout=5)
            except Exception as e:
                self.get_logger().warn(f'Error closing ffmpeg: {str(e)}')
                self.ffmpeg_process.kill()
        
        self.get_logger().info(f'Total frames streamed: {self.frame_count}')
        self.get_logger().info('RTMP Streamer stopped')


def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    print('\nStopping RTMP streamer...')
    sys.exit(0)


def main(args=None):
    # Setup signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    rclpy.init(args=args)
    
    streamer = None
    try:
        streamer = RTMPStreamer()
        rclpy.spin(streamer)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in RTMP streamer: {str(e)}')
    finally:
        if streamer is not None:
            streamer.cleanup()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

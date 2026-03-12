#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, String
import subprocess
import re
import json


class SeanoCommunicationMonitorNode(Node):
    def __init__(self):
        super().__init__('seano_communication_monitor')
        
        # Publishers - Aggregate
        self.comm_status_pub = self.create_publisher(String, '/seano/communication/status', 10)
        self.comm_failure_alert_pub = self.create_publisher(Bool, '/seano/communication/failure_alert', 10)
        
        # Publishers - WiFi
        self.wifi_rssi_pub = self.create_publisher(Float32, '/seano/communication/wifi/rssi', 10)
        self.wifi_quality_pub = self.create_publisher(Float32, '/seano/communication/wifi/quality', 10)
        self.wifi_status_pub = self.create_publisher(String, '/seano/communication/wifi/status', 10)
        
        # Publishers - GSM
        self.gsm_signal_pub = self.create_publisher(Float32, '/seano/communication/gsm/signal', 10)
        self.gsm_quality_pub = self.create_publisher(Float32, '/seano/communication/gsm/quality', 10)
        self.gsm_status_pub = self.create_publisher(String, '/seano/communication/gsm/status', 10)
        
        # Publishers - Ethernet
        self.ethernet_status_pub = self.create_publisher(String, '/seano/communication/ethernet/status', 10)
        self.ethernet_link_pub = self.create_publisher(Bool, '/seano/communication/ethernet/link', 10)
        
        # Subscribe to communication node status
        self.comm_node_sub = self.create_subscription(
            String,
            '/communication/status',
            self.comm_node_callback,
            10
        )
        
        # Parameters
        self.declare_parameter('failsafe.communication.check_interval', 2.0)
        self.declare_parameter('failsafe.communication.wifi_interface', 'wlP1p1s0')
        self.declare_parameter('failsafe.communication.gsm_interface', 'wwan0')
        self.declare_parameter('failsafe.communication.ethernet_interface', 'enP8p1s0')
        self.declare_parameter('failsafe.communication.wifi_rssi_warning', -70.0)
        self.declare_parameter('failsafe.communication.wifi_rssi_critical', -80.0)
        self.declare_parameter('failsafe.communication.gsm_signal_warning', 15.0)  # CSQ/RSSI value
        self.declare_parameter('failsafe.communication.gsm_signal_critical', 10.0)
        self.declare_parameter('failsafe.communication.consecutive_failures', 3)
        
        self.check_interval = self.get_parameter('failsafe.communication.check_interval').value
        self.wifi_if = self.get_parameter('failsafe.communication.wifi_interface').value
        self.gsm_if = self.get_parameter('failsafe.communication.gsm_interface').value
        self.ethernet_if = self.get_parameter('failsafe.communication.ethernet_interface').value
        self.wifi_rssi_warning = self.get_parameter('failsafe.communication.wifi_rssi_warning').value
        self.wifi_rssi_critical = self.get_parameter('failsafe.communication.wifi_rssi_critical').value
        self.gsm_signal_warning = self.get_parameter('failsafe.communication.gsm_signal_warning').value
        self.gsm_signal_critical = self.get_parameter('failsafe.communication.gsm_signal_critical').value
        self.max_failures = self.get_parameter('failsafe.communication.consecutive_failures').value
        
        # State variables
        self.wifi_rssi = -100.0
        self.wifi_quality = 0.0
        self.wifi_connected = False
        
        self.gsm_signal = 0.0
        self.gsm_quality = 0.0
        self.gsm_connected = False
        
        self.ethernet_connected = False
        
        self.consecutive_failures = 0
        self.last_active_link = None
        self.all_links_down = False
        
        # Timer
        self.timer = self.create_timer(self.check_interval, self.check_all_communication)
        
        self.get_logger().info('SEANO Communication Monitor Node Started')
        self.get_logger().info(f'WiFi Interface: {self.wifi_if}')
        self.get_logger().info(f'GSM Interface: {self.gsm_if}')
        self.get_logger().info(f'Ethernet Interface: {self.ethernet_if}')
        self.get_logger().info(f'WiFi RSSI Thresholds: Warning {self.wifi_rssi_warning}dBm | Critical {self.wifi_rssi_critical}dBm')
        self.get_logger().info(f'GSM Signal Thresholds: Warning {self.gsm_signal_warning} | Critical {self.gsm_signal_critical}')
    
    def comm_node_callback(self, msg):
        """Track active link from communication node"""
        if msg.data.startswith('SWITCHED_TO:'):
            self.last_active_link = msg.data.split(':')[1]
    
    def comm_node_callback(self, msg):
        """Track active link from communication node"""
        if msg.data.startswith('SWITCHED_TO:'):
            self.last_active_link = msg.data.split(':')[1]
    
    def check_all_communication(self):
        """Check all communication interfaces"""
        # Check WiFi
        self.check_wifi()
        
        # Check GSM
        self.check_gsm()
        
        # Check Ethernet
        self.check_ethernet()
        
        # Determine overall communication status
        self.evaluate_overall_status()
        
        # Publish aggregate status
        self.publish_aggregate_status()
    
    def check_wifi(self):
        """Check WiFi signal strength (RSSI)"""
        try:
            result = subprocess.run(
                ['iwconfig', self.wifi_if],
                capture_output=True,
                text=True,
                timeout=2
            )
            
            if result.returncode == 0:
                output = result.stdout
                
                # Parse RSSI
                rssi_match = re.search(r'Signal level[=:](-?\d+)\s*dBm', output)
                if rssi_match:
                    self.wifi_rssi = float(rssi_match.group(1))
                    self.wifi_connected = True
                else:
                    # Try alternative format
                    rssi_match = re.search(r'Signal level[=:](-?\d+)/\d+', output)
                    if rssi_match:
                        raw_signal = float(rssi_match.group(1))
                        self.wifi_rssi = -100 + (raw_signal * 1.5)
                        self.wifi_connected = True
                    else:
                        self.wifi_connected = False
                
                # Parse Link Quality
                quality_match = re.search(r'Link Quality[=:](\d+)/(\d+)', output)
                if quality_match:
                    quality_current = float(quality_match.group(1))
                    quality_max = float(quality_match.group(2))
                    self.wifi_quality = (quality_current / quality_max) * 100.0
                else:
                    self.wifi_quality = self.rssi_to_quality(self.wifi_rssi)
            else:
                self.wifi_connected = False
                
        except Exception as e:
            self.get_logger().debug(f'WiFi check error: {str(e)}')
            self.wifi_connected = False
        
        # Publish WiFi data
        rssi_msg = Float32()
        rssi_msg.data = self.wifi_rssi
        self.wifi_rssi_pub.publish(rssi_msg)
        
        quality_msg = Float32()
        quality_msg.data = self.wifi_quality
        self.wifi_quality_pub.publish(quality_msg)
        
        # Determine WiFi status
        if not self.wifi_connected:
            wifi_status = 'disconnected'
        elif self.wifi_rssi <= self.wifi_rssi_critical:
            wifi_status = 'critical'
        elif self.wifi_rssi <= self.wifi_rssi_warning:
            wifi_status = 'weak'
        else:
            wifi_status = 'good'
        
        status_msg = String()
        status_msg.data = wifi_status
        self.wifi_status_pub.publish(status_msg)
    
    def check_gsm(self):
        """Check GSM signal strength"""
        try:
            # Try mmcli (ModemManager) first
            result = subprocess.run(
                ['mmcli', '-m', '0', '--signal-get'],
                capture_output=True,
                text=True,
                timeout=2
            )
            
            if result.returncode == 0:
                output = result.stdout
                
                # Parse signal strength (dBm or percentage)
                signal_match = re.search(r'signal strength:\s*(\d+)\s*%', output)
                if signal_match:
                    self.gsm_signal = float(signal_match.group(1))
                    self.gsm_quality = self.gsm_signal
                    self.gsm_connected = True
                else:
                    # Try RSSI format
                    rssi_match = re.search(r'rssi:\s*(-?\d+(?:\.\d+)?)\s*dBm', output)
                    if rssi_match:
                        rssi = float(rssi_match.group(1))
                        # Convert dBm to CSQ (0-31 scale)
                        self.gsm_signal = max(0, min(31, (rssi + 113) / 2))
                        self.gsm_quality = (self.gsm_signal / 31) * 100
                        self.gsm_connected = True
                    else:
                        self.gsm_connected = False
            else:
                # Try alternative: AT commands via qmicli or direct serial
                self.gsm_connected = False
                
        except Exception as e:
            self.get_logger().debug(f'GSM check error: {str(e)}')
            self.gsm_connected = False
        
        # Publish GSM data
        signal_msg = Float32()
        signal_msg.data = self.gsm_signal
        self.gsm_signal_pub.publish(signal_msg)
        
        quality_msg = Float32()
        quality_msg.data = self.gsm_quality
        self.gsm_quality_pub.publish(quality_msg)
        
        # Determine GSM status
        if not self.gsm_connected:
            gsm_status = 'disconnected'
        elif self.gsm_signal <= self.gsm_signal_critical:
            gsm_status = 'critical'
        elif self.gsm_signal <= self.gsm_signal_warning:
            gsm_status = 'weak'
        else:
            gsm_status = 'good'
        
        status_msg = String()
        status_msg.data = gsm_status
        self.gsm_status_pub.publish(status_msg)
    
    def check_ethernet(self):
        """Check Ethernet link status"""
        try:
            # Check if interface is up
            result = subprocess.run(
                ['cat', f'/sys/class/net/{self.ethernet_if}/operstate'],
                capture_output=True,
                text=True,
                timeout=1
            )
            
            if result.returncode == 0:
                state = result.stdout.strip().lower()
                self.ethernet_connected = (state == 'up')
            else:
                self.ethernet_connected = False
                
        except Exception as e:
            self.get_logger().debug(f'Ethernet check error: {str(e)}')
            self.ethernet_connected = False
        
        # Publish Ethernet data
        link_msg = Bool()
        link_msg.data = self.ethernet_connected
        self.ethernet_link_pub.publish(link_msg)
        
        status_msg = String()
        status_msg.data = 'connected' if self.ethernet_connected else 'disconnected'
        self.ethernet_status_pub.publish(status_msg)
    
    def evaluate_overall_status(self):
        """Evaluate overall communication status"""
        # Check if at least one link is available
        has_active_link = self.wifi_connected or self.gsm_connected or self.ethernet_connected
        
        if not has_active_link:
            self.consecutive_failures += 1
            self.all_links_down = (self.consecutive_failures >= self.max_failures)
        else:
            self.consecutive_failures = 0
            self.all_links_down = False
    
    def publish_aggregate_status(self):
        """Publish aggregate communication status"""
        # Determine overall status
        if self.all_links_down:
            overall_status = 'all_down'
        elif self.ethernet_connected:
            overall_status = 'ethernet_active'
        elif self.wifi_connected and self.wifi_rssi > self.wifi_rssi_warning:
            overall_status = 'wifi_good'
        elif self.wifi_connected:
            overall_status = 'wifi_weak'
        elif self.gsm_connected and self.gsm_signal > self.gsm_signal_warning:
            overall_status = 'gsm_good'
        elif self.gsm_connected:
            overall_status = 'gsm_weak'
        else:
            overall_status = 'degraded'
        
        # Publish status
        status_msg = String()
        status_msg.data = overall_status
        self.comm_status_pub.publish(status_msg)
        
        # Publish failure alert
        failure_alert_msg = Bool()
        failure_alert_msg.data = self.all_links_down
        self.comm_failure_alert_pub.publish(failure_alert_msg)
        
        # Log warnings
        if self.all_links_down:
            self.get_logger().error('ALL COMMUNICATION LINKS DOWN - FAILSAFE REQUIRED!')
        elif overall_status == 'degraded':
            self.get_logger().warn('Communication degraded - limited connectivity')
        
        # Log info periodically (every 10 seconds)
        if int(self.get_clock().now().seconds_nanoseconds()[0]) % 10 == 0:
            self.get_logger().info(
                f'Communication: WiFi {self.wifi_rssi:.1f}dBm | '
                f'GSM {self.gsm_signal:.0f} | '
                f'Ethernet {"UP" if self.ethernet_connected else "DOWN"} | '
                f'Status: {overall_status.upper()}'
            )
    
    def rssi_to_quality(self, rssi):
        """Convert RSSI to quality percentage"""
        if rssi >= -50:
            return 100.0
        elif rssi <= -100:
            return 0.0
        else:
            return ((rssi + 100) / 50) * 100.0


def main(args=None):
    rclpy.init(args=args)
    node = SeanoCommunicationMonitorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

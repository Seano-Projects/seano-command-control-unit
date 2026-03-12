#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import time
import requests
from ping3 import ping


class CommunicationNode(Node):
    """
    ROS2 node for managing network communication between GSM, WiFi and Ethernet.
    Automatically switches to the best available network based on latency and bandwidth.
    """

    def __init__(self):
        super().__init__('communication')

        # Declare parameters with default values
        self.declare_parameter('communication.gsm_interface', 'wwan0')
        self.declare_parameter('communication.wifi_interface', 'wlP1p1s0')
        self.declare_parameter('communication.ethernet_interface', 'enP8p1s0')
        self.declare_parameter('communication.ping_target', '8.8.8.8')
        self.declare_parameter('communication.speed_test_url', 'http://speedtest.tele2.net/1MB.zip')
        self.declare_parameter('communication.latency_threshold', 200.0)
        self.declare_parameter('communication.bandwidth_threshold', 1.0)
        self.declare_parameter('communication.check_interval', 5.0)

        # Get parameter values
        self.gsm_if = self.get_parameter('communication.gsm_interface').value
        self.wifi_if = self.get_parameter('communication.wifi_interface').value
        self.ethernet_if = self.get_parameter('communication.ethernet_interface').value
        self.target = self.get_parameter('communication.ping_target').value
        self.speed_test_url = self.get_parameter('communication.speed_test_url').value
        self.latency_threshold = self.get_parameter('communication.latency_threshold').value
        self.bandwidth_threshold = self.get_parameter('communication.bandwidth_threshold').value
        self.check_interval = self.get_parameter('communication.check_interval').value

        # Current active link
        self.current_link = None

        # Publisher for network status
        self.status_publisher = self.create_publisher(
            String,
            'communication/status',
            10
        )

        # Create timer for periodic network checking
        self.timer = self.create_timer(
            self.check_interval,
            self.check_and_switch_network
        )

        self.get_logger().info(f'Communication node started')
        self.get_logger().info(f'GSM Interface: {self.gsm_if}')
        self.get_logger().info(f'WiFi Interface: {self.wifi_if}')
        self.get_logger().info(f'Ethernet Interface: {self.ethernet_if}')
        self.get_logger().info(f'Ping Target: {self.target}')
        self.get_logger().info(f'Speed Test URL: {self.speed_test_url}')
        self.get_logger().info(f'Latency Threshold: {self.latency_threshold}ms')
        self.get_logger().info(f'Bandwidth Threshold: {self.bandwidth_threshold}Mbps')
        self.get_logger().info(f'Check Interval: {self.check_interval}s')

    def check_latency(self, interface):
        """
        Check network latency for a given interface.
        
        Args:
            interface: Network interface name (e.g., 'wwan0', 'wlan0')
            
        Returns:
            Latency in milliseconds, or None if unreachable
        """
        try:
            latency = ping(self.target, interface=interface, timeout=2)
            if latency is None:
                return None
            return latency * 1000  # Convert to milliseconds
        except Exception as e:
            self.get_logger().warning(f'Ping failed on {interface}: {e}')
            return None

    def check_bandwidth(self, interface):
        """
        Check download bandwidth for a given interface.
        
        Args:
            interface: Network interface name
            
        Returns:
            Bandwidth in Mbps, or None if test failed
        """
        try:
            # Bind socket to specific interface
            import socket
            old_getaddrinfo = socket.getaddrinfo
            
            def custom_getaddrinfo(*args, **kwargs):
                family, socktype, proto, canonname, sockaddr = old_getaddrinfo(*args, **kwargs)[0]
                s = socket.socket(family, socktype, proto)
                s.setsockopt(socket.SOL_SOCKET, 25, (interface + '\0').encode('utf-8'))  # SO_BINDTODEVICE = 25
                return [(family, socktype, proto, canonname, sockaddr)]
            
            socket.getaddrinfo = custom_getaddrinfo
            
            # Download test file
            start_time = time.time()
            response = requests.get(self.speed_test_url, timeout=30, stream=True)
            
            if response.status_code != 200:
                socket.getaddrinfo = old_getaddrinfo
                return None
            
            # Calculate download size
            total_size = 0
            for chunk in response.iter_content(chunk_size=8192):
                total_size += len(chunk)
            
            elapsed_time = time.time() - start_time
            socket.getaddrinfo = old_getaddrinfo
            
            # Calculate bandwidth in Mbps
            if elapsed_time > 0:
                bandwidth_mbps = (total_size * 8) / (elapsed_time * 1_000_000)
                return round(bandwidth_mbps, 2)
            
            return None
            
        except Exception as e:
            self.get_logger().warning(f'Bandwidth test failed on {interface}: {e}')
            return None

    def switch_route(self, interface):
        """
        Switch the default route to the specified interface.
        
        Args:
            interface: Network interface to switch to
        """
        self.get_logger().info(f'Switching route to {interface}')
        try:
            os.system(f'sudo ip route replace default dev {interface}')
            
            # Publish status
            msg = String()
            msg.data = f'SWITCHED_TO:{interface}'
            self.status_publisher.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Failed to switch route: {e}')

    def check_and_switch_network(self):
        """
        Periodic callback to check network latency, bandwidth and switch if necessary.
        """
        # Check latency and bandwidth on all interfaces
        self.get_logger().info('=' * 60)
        self.get_logger().info('Network Quality Check')
        self.get_logger().info('-' * 60)
        
        # GSM checks
        gsm_latency = self.check_latency(self.gsm_if)
        gsm_bandwidth = self.check_bandwidth(self.gsm_if) if gsm_latency else None
        
        # WiFi checks
        wifi_latency = self.check_latency(self.wifi_if)
        wifi_bandwidth = self.check_bandwidth(self.wifi_if) if wifi_latency else None
        
        # Ethernet checks
        ethernet_latency = self.check_latency(self.ethernet_if)
        ethernet_bandwidth = self.check_bandwidth(self.ethernet_if) if ethernet_latency else None

        # Display results
        self.get_logger().info(f'GSM ({self.gsm_if}):')
        self.get_logger().info(f'  • Latency: {gsm_latency:.2f}ms' if gsm_latency else f'  • Latency: unreachable')
        self.get_logger().info(f'  • Bandwidth: {gsm_bandwidth:.2f}Mbps' if gsm_bandwidth else f'  • Bandwidth: N/A')
        
        self.get_logger().info(f'WiFi ({self.wifi_if}):')
        self.get_logger().info(f'  • Latency: {wifi_latency:.2f}ms' if wifi_latency else f'  • Latency: unreachable')
        self.get_logger().info(f'  • Bandwidth: {wifi_bandwidth:.2f}Mbps' if wifi_bandwidth else f'  • Bandwidth: N/A')
        
        self.get_logger().info(f'Ethernet ({self.ethernet_if}):')
        self.get_logger().info(f'  • Latency: {ethernet_latency:.2f}ms' if ethernet_latency else f'  • Latency: unreachable')
        self.get_logger().info(f'  • Bandwidth: {ethernet_bandwidth:.2f}Mbps' if ethernet_bandwidth else f'  • Bandwidth: N/A')

        selected_link = None
        selection_reason = ""

        # Decision logic: Prefer Ethernet > WiFi > GSM
        # Consider both latency and bandwidth
        candidates = []
        
        if ethernet_latency and ethernet_bandwidth and ethernet_bandwidth >= self.bandwidth_threshold:
            candidates.append(('ethernet', self.ethernet_if, ethernet_latency, ethernet_bandwidth))
        
        if wifi_latency and wifi_bandwidth and wifi_bandwidth >= self.bandwidth_threshold:
            candidates.append(('wifi', self.wifi_if, wifi_latency, wifi_bandwidth))
        
        if gsm_latency and gsm_latency < self.latency_threshold:
            candidates.append(('gsm', self.gsm_if, gsm_latency, gsm_bandwidth or 0))

        if candidates:
            # Sort by bandwidth (highest first), then by latency (lowest first)
            candidates.sort(key=lambda x: (-x[3], x[2]))
            selected = candidates[0]
            selected_link = selected[1]
            selection_reason = f'{selected[0].upper()} - {selected[3]:.2f}Mbps @ {selected[2]:.2f}ms'
        else:
            self.get_logger().warning('No network available!')

        self.get_logger().info('-' * 60)
        if selected_link:
            self.get_logger().info(f'Selected: {selection_reason}')
        
        # Switch route if the selected link is different from current
        if selected_link and selected_link != self.current_link:
            self.switch_route(selected_link)
            self.current_link = selected_link
        elif selected_link == self.current_link:
            self.get_logger().info(f'Already using {self.current_link}')
        
        self.get_logger().info('=' * 60)


def main(args=None):
    rclpy.init(args=args)
    
    node = CommunicationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

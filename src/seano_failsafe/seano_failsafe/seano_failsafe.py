#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float32
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandLong
import json
import time


class SeanoFailsafeNode(Node):
    def __init__(self):
        super().__init__('seano_failsafe')
        
        # Subscribers - Battery (dari seano_battery node)
        self.battery_low_sub = self.create_subscription(
            Bool,
            '/seano/battery/low_alert',
            self.battery_callback,
            10
        )
        self.battery_voltage_sub = self.create_subscription(
            Float32,
            '/seano/battery/voltage',
            self.battery_voltage_callback,
            10
        )
        
        # Subscribers - Communication (dari seano_communication_monitor node)
        self.comm_failure_sub = self.create_subscription(
            Bool,
            '/seano/communication/failure_alert',
            self.communication_callback,
            10
        )
        self.comm_status_sub = self.create_subscription(
            String,
            '/seano/communication/status',
            self.comm_status_callback,
            10
        )
        
        # Subscribers - Mavros
        self.mavros_state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.mavros_state_callback,
            10
        )
        
        # Publishers
        self.failsafe_status_pub = self.create_publisher(String, '/seano/failsafe/status', 10)
        self.emergency_stop_pub = self.create_publisher(Bool, '/seano/failsafe/emergency_stop', 10)
        self.failsafe_event_pub = self.create_publisher(String, '/seano/failsafe/event', 10)
        self.mqtt_notify_pub = self.create_publisher(String, 'failsafe/alert', 10)
        
        # Service clients for Mavros
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.command_client = self.create_client(CommandLong, '/mavros/cmd/command')
        
        # Parameters
        self.declare_parameter('failsafe.system.battery_failsafe_enabled', True)
        self.declare_parameter('failsafe.system.communication_failsafe_enabled', True)
        self.declare_parameter('failsafe.system.failsafe_mode', 'RTL')  # RTL, LOITER, LAND
        self.declare_parameter('failsafe.system.notification_delay', 2.0)  # seconds before action
        self.declare_parameter('failsafe.system.recovery_delay', 10.0)  # seconds to wait for recovery
        self.declare_parameter('failsafe.system.mode_enforce_interval', 2.0)  # seconds
        
        self.battery_failsafe_enabled = self.get_parameter('failsafe.system.battery_failsafe_enabled').value
        self.comm_failsafe_enabled = self.get_parameter('failsafe.system.communication_failsafe_enabled').value
        self.failsafe_mode = self.get_parameter('failsafe.system.failsafe_mode').value
        self.notification_delay = self.get_parameter('failsafe.system.notification_delay').value
        self.recovery_delay = self.get_parameter('failsafe.system.recovery_delay').value
        self.mode_enforce_interval = self.get_parameter('failsafe.system.mode_enforce_interval').value
        
        # State variables
        self.battery_critical = False
        self.comm_critical = False
        self.failsafe_active = False
        self.failsafe_triggered_time = None
        self.recovery_started_time = None
        self.mode_changed = False
        self.last_mode_enforce_time = 0.0
        
        self.current_voltage = 0.0
        self.comm_status = 'unknown'
        self.current_mavros_mode = 'UNKNOWN'
        self.mavros_connected = False
        
        # Timer for status updates
        self.status_timer = self.create_timer(1.0, self.check_failsafe_conditions)
        
        self.get_logger().info('SEANO Failsafe Node Started')
        self.get_logger().info(f'Battery Failsafe: {self.battery_failsafe_enabled}')
        self.get_logger().info(f'Communication Failsafe: {self.comm_failsafe_enabled}')
        self.get_logger().info(f'Failsafe Mode: {self.failsafe_mode}')
        self.get_logger().info(f'Notification Delay: {self.notification_delay}s')
        self.get_logger().info('SEANO Failsafe Node Started')
        self.get_logger().info(f'Battery Failsafe: {self.battery_failsafe_enabled}')
        self.get_logger().info(f'Communication Failsafe: {self.comm_failsafe_enabled}')
        self.get_logger().info(f'Failsafe Mode: {self.failsafe_mode}')
        self.get_logger().info(f'Notification Delay: {self.notification_delay}s')
    
    def battery_voltage_callback(self, msg):
        """Track current battery voltage"""
        self.current_voltage = msg.data
    
    def battery_callback(self, msg):
        """Handle battery low alert dari seano_battery node"""
        if not self.battery_failsafe_enabled:
            return
        self.battery_critical = msg.data
    
    def comm_status_callback(self, msg):
        """Track current communication status"""
        self.comm_status = msg.data
    
    def communication_callback(self, msg):
        """Handle communication failure alert dari seano_communication_monitor node"""
        if not self.comm_failsafe_enabled:
            return
        self.comm_critical = msg.data
    
    def mavros_state_callback(self, msg):
        """Track Mavros state"""
        self.mavros_connected = msg.connected
        self.current_mavros_mode = msg.mode
    
    def check_failsafe_conditions(self):
        """Check if failsafe should be triggered"""
        failsafe_needed = self.battery_critical or self.comm_critical
        
        if failsafe_needed and not self.failsafe_active:
            self.recovery_started_time = None
            # Start failsafe procedure
            if self.failsafe_triggered_time is None:
                self.failsafe_triggered_time = time.time()
                self.get_logger().warn('FAILSAFE CONDITION DETECTED - Starting notification delay...')
                
                # Send immediate notification
                self.send_mqtt_notification('warning')
            
            # Check if delay has passed
            elapsed = time.time() - self.failsafe_triggered_time
            if elapsed >= self.notification_delay:
                self.activate_failsafe()

        elif failsafe_needed and self.failsafe_active:
            # While condition is still critical, keep failsafe mode enforced.
            self.recovery_started_time = None
            self.enforce_failsafe_mode()
        
        elif not failsafe_needed and self.failsafe_active:
            # Check if conditions have been good for recovery_delay
            if self.recovery_started_time is None:
                self.recovery_started_time = time.time()
                self.get_logger().info('Failsafe condition cleared, waiting recovery delay...')
            else:
                elapsed_recovery = time.time() - self.recovery_started_time
                if elapsed_recovery >= self.recovery_delay:
                    self.deactivate_failsafe()
        
        elif not failsafe_needed and self.failsafe_triggered_time is not None:
            # Condition cleared before triggering
            self.get_logger().info('Failsafe condition cleared before trigger')
            self.failsafe_triggered_time = None
        
        # Publish status
        self.publish_status()
    
    def activate_failsafe(self):
        """Activate failsafe procedures"""
        if self.failsafe_active:
            return
        
        self.failsafe_active = True
        self.last_mode_enforce_time = 0.0
        self.get_logger().error('=' * 60)
        self.get_logger().error('FAILSAFE ACTIVATED!')
        self.get_logger().error('=' * 60)
        
        # Determine trigger reason
        reasons = []
        if self.battery_critical:
            reasons.append(f'Battery: {self.current_voltage:.2f}V')
        if self.comm_critical:
            reasons.append(f'Communication: {self.comm_status}')
        
        reason_str = ' | '.join(reasons)
        self.get_logger().error(f'Trigger Reason: {reason_str}')
        
        # Send MQTT notification
        self.send_mqtt_notification('critical', reason_str)
        
        # Wait a moment for notification to be sent
        time.sleep(0.5)
        
        # Change flight mode immediately, then keep enforcing while active.
        self.enforce_failsafe_mode(force=True)
        
        # Publish emergency stop
        emergency_msg = Bool()
        emergency_msg.data = True
        self.emergency_stop_pub.publish(emergency_msg)
        
        # Publish failsafe event
        event_msg = String()
        event_data = {
            'event': 'failsafe_activated',
            'timestamp': time.time(),
            'reason': reason_str,
            'mode_set': self.failsafe_mode
        }
        event_msg.data = json.dumps(event_data)
        self.failsafe_event_pub.publish(event_msg)
    
    def deactivate_failsafe(self):
        """Deactivate failsafe (conditions recovered)"""
        if not self.failsafe_active:
            return
        
        self.failsafe_active = False
        self.failsafe_triggered_time = None
        self.recovery_started_time = None
        self.mode_changed = False
        
        self.get_logger().info('Failsafe deactivated - conditions recovered')
        
        # Send recovery notification
        self.send_mqtt_notification('recovery')
        
        # Cancel emergency stop
        emergency_msg = Bool()
        emergency_msg.data = False
        self.emergency_stop_pub.publish(emergency_msg)
        
        # Publish failsafe event
        event_msg = String()
        event_data = {
            'event': 'failsafe_deactivated',
            'timestamp': time.time()
        }
        event_msg.data = json.dumps(event_data)
        self.failsafe_event_pub.publish(event_msg)
    
    def send_mqtt_notification(self, severity, reason=''):
        """Send failsafe notification via MQTT"""
        notification = {
            'type': 'failsafe',
            'severity': severity,  # warning, critical, recovery
            'timestamp': time.time(),
            'vehicle_id': 'USV-001',  # TODO: get from params
            'battery': {
                'voltage': self.current_voltage,
                'critical': self.battery_critical
            },
            'communication': {
                'status': self.comm_status,
                'critical': self.comm_critical
            },
            'mavros': {
                'mode': self.current_mavros_mode,
                'connected': self.mavros_connected
            }
        }
        
        if reason:
            notification['reason'] = reason
        
        if severity == 'critical':
            notification['action'] = f'Flight mode changed to {self.failsafe_mode}'
        
        msg = String()
        msg.data = json.dumps(notification)
        self.mqtt_notify_pub.publish(msg)
        
        self.get_logger().info(f'MQTT Notification sent: {severity.upper()}')
    
    def change_flight_mode(self, mode):
        """Change Mavros flight mode"""
        if not self.mavros_connected:
            self.get_logger().error('Cannot change mode - Mavros not connected!')
            return False
        
        if str(self.current_mavros_mode).upper() == str(mode).upper():
            self.mode_changed = True
            return True
        
        self.get_logger().warn(f'Changing flight mode to {mode}...')
        
        if not self.set_mode_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('SetMode service not available!')
            return False
        
        try:
            request = SetMode.Request()
            request.custom_mode = mode
            
            future = self.set_mode_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is not None and future.result().mode_sent:
                self.get_logger().warn(f'Flight mode changed to {mode} successfully!')
                self.mode_changed = True
                return True
            else:
                self.get_logger().error(f'Failed to change flight mode to {mode}')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Error changing flight mode: {str(e)}')
            return False

    def enforce_failsafe_mode(self, force=False):
        """Keep FCU in failsafe mode while failsafe condition remains active."""
        if not self.failsafe_active:
            return

        if str(self.current_mavros_mode).upper() == str(self.failsafe_mode).upper():
            self.mode_changed = True
            return

        now = time.time()
        if (not force) and ((now - self.last_mode_enforce_time) < self.mode_enforce_interval):
            return

        self.last_mode_enforce_time = now
        self.change_flight_mode(self.failsafe_mode)
    
    def publish_status(self):
        """Publish failsafe status"""
        status_msg = String()
        if self.failsafe_active:
            status_msg.data = 'ACTIVE'
        elif self.failsafe_triggered_time is not None:
            remaining = self.notification_delay - (time.time() - self.failsafe_triggered_time)
            status_msg.data = f'PENDING ({remaining:.1f}s)'
        else:
            status_msg.data = 'INACTIVE'
        
        self.failsafe_status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SeanoFailsafeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

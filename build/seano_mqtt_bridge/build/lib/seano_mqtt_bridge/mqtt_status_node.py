#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
import ssl
import time


class MqttStatusNode(Node):
    """
    ROS2 node untuk publish status online/offline ke MQTT broker
    dengan heartbeat realtime setiap 30 detik.
    """

    def __init__(self):
        super().__init__('mqtt_status')

        # ================= PARAMETERS =================
        self.declare_parameter('vehicle_code', 'USV-001')
        self.declare_parameter('mqtt.broker', 'mqtt.seano.cloud')
        self.declare_parameter('mqtt.port', 8883)
        self.declare_parameter('mqtt.username', 'seanomqtt')
        self.declare_parameter('mqtt.password', 'Seano2025*')
        self.declare_parameter('mqtt.keepalive', 60)
        self.declare_parameter('mqtt.base_topic', 'seano')
        self.declare_parameter('heartbeat_interval', 30.0)

        # Get parameters
        self.vehicle_code = self.get_parameter('vehicle_code').value
        self.mqtt_broker = self.get_parameter('mqtt.broker').value
        self.mqtt_port = int(self.get_parameter('mqtt.port').value)
        self.mqtt_username = self.get_parameter('mqtt.username').value
        self.mqtt_password = self.get_parameter('mqtt.password').value
        self.keepalive = int(self.get_parameter('mqtt.keepalive').value)
        self.base_topic = self.get_parameter('mqtt.base_topic').value
        self.heartbeat_interval = self.get_parameter('heartbeat_interval').value

        # Status topic
        self.status_topic = f"{self.base_topic}/{self.vehicle_code}/status"

        self.get_logger().info(f"Vehicle Code: {self.vehicle_code}")
        self.get_logger().info(f"Status Topic: {self.status_topic}")
        self.get_logger().info(f"MQTT Broker: {self.mqtt_broker}:{self.mqtt_port}")
        self.get_logger().info(f"Keepalive: {self.keepalive}s, Heartbeat: {self.heartbeat_interval}s")

        # ================= MQTT CLIENT SETUP =================
        # Cek versi paho-mqtt untuk kompatibilitas
        try:
            # Paho MQTT v2.x uses CallbackAPIVersion
            from paho.mqtt.client import CallbackAPIVersion
            self.client = mqtt.Client(
                callback_api_version=CallbackAPIVersion.VERSION1,
                client_id=f"{self.vehicle_code}-ros2"
            )
            self.get_logger().info("Using Paho MQTT v2.x (CallbackAPIVersion.VERSION1)")
        except ImportError:
            # Paho MQTT v1.x
            self.client = mqtt.Client(client_id=f"{self.vehicle_code}-ros2")
            self.get_logger().info("Using Paho MQTT v1.x")
        
        # Set username & password
        if self.mqtt_username:
            self.client.username_pw_set(self.mqtt_username, self.mqtt_password)

        # TLS setup (insecure untuk testing)
        try:
            self.client.tls_set(cert_reqs=ssl.CERT_NONE)
            self.client.tls_insecure_set(True)
        except Exception as e:
            self.get_logger().error(f"TLS setup failed: {e}")

        # Last Will and Testament (LWT) - otomatis publish "offline" saat disconnect
        self.client.will_set(
            self.status_topic,
            payload="offline",
            qos=1,
            retain=True
        )

        # Set callbacks
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_connect_fail = self.on_connect_fail
        
        # Enable MQTT logging untuk debugging
        self.client.enable_logger(logger=None)  # Use Python's logging module

        # ================= CONNECT TO MQTT =================
        self.connected = False
        
        # Start MQTT loop thread SEBELUM connect
        self.client.loop_start()
        self.get_logger().info("🔄 MQTT loop thread started")
        
        # Connect to broker
        self.connect_mqtt()

        # ================= HEARTBEAT TIMER =================
        # Timer untuk publish heartbeat setiap 30 detik
        self.heartbeat_timer = self.create_timer(
            self.heartbeat_interval,
            self.heartbeat_callback
        )

    def connect_mqtt(self):
        """Connect ke MQTT broker"""
        try:
            self.get_logger().info(f"🔌 Connecting to MQTT broker {self.mqtt_broker}:{self.mqtt_port}...")
            self.client.connect(
                self.mqtt_broker,
                self.mqtt_port,
                keepalive=self.keepalive
            )
        except ConnectionRefusedError as e:
            self.get_logger().error(f"❌ Connection refused: {e}")
        except TimeoutError as e:
            self.get_logger().error(f"❌ Connection timeout: {e}")
        except OSError as e:
            self.get_logger().error(f"❌ Network error: {e}")
        except Exception as e:
            self.get_logger().error(f"❌ MQTT connection failed: {type(e).__name__}: {e}")
            self.get_logger().warn("⏳ Will retry connecting...")

    def on_connect(self, client, userdata, flags, rc):
        """Callback saat MQTT connected"""
        if rc == 0:
            self.connected = True
            self.get_logger().info("✅ MQTT connected successfully")
            
            # Publish status "online" dengan retain=True
            result = self.client.publish(
                self.status_topic,
                payload="online",
                qos=1,
                retain=True
            )
            
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                self.get_logger().info(f"📤 Published: {self.status_topic} = online")
            else:
                self.get_logger().error(f"❌ Failed to publish online status (rc={result.rc})")
        else:
            self.connected = False
            error_messages = {
                1: "Connection refused - incorrect protocol version",
                2: "Connection refused - invalid client identifier",
                3: "Connection refused - server unavailable",
                4: "Connection refused - bad username or password",
                5: "Connection refused - not authorized"
            }
            error_msg = error_messages.get(rc, f"Unknown error code: {rc}")
            self.get_logger().error(f"❌ MQTT connection failed: {error_msg}")

    def on_connect_fail(self, client, userdata):
        """Callback saat MQTT connection fail"""
        self.connected = False
        self.get_logger().error("❌ MQTT connection failed - cannot reach broker")

    def on_disconnect(self, client, userdata, rc):
        """Callback saat MQTT disconnected"""
        self.connected = False
        if rc != 0:
            self.get_logger().warn(f"⚠️  MQTT disconnected unexpectedly (rc={rc}), auto-reconnecting...")
        else:
            self.get_logger().info("MQTT disconnected gracefully")

    def heartbeat_callback(self):
        """Callback untuk publish heartbeat setiap 30 detik"""
        if self.connected:
            result = self.client.publish(
                self.status_topic,
                payload="online",
                qos=1,
                retain=True
            )
            
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                self.get_logger().info(f"💓 Heartbeat sent: {self.status_topic} = online")
            else:
                self.get_logger().warn(f"⚠️  Failed to send heartbeat (rc={result.rc})")
        else:
            self.get_logger().warn("⚠️  Heartbeat skipped - MQTT not connected")

    def destroy_node(self):
        """Cleanup saat node di-shutdown"""
        self.get_logger().info("🛑 Shutting down MQTT status node...")
        
        # Publish offline status sebelum disconnect
        if self.connected:
            try:
                self.client.publish(
                    self.status_topic,
                    payload="offline",
                    qos=1,
                    retain=True
                )
                self.get_logger().info("📤 Published offline status")
            except Exception as e:
                self.get_logger().error(f"Failed to publish offline status: {e}")
        
        # Disconnect MQTT
        try:
            self.client.disconnect()
            self.client.loop_stop()
        except Exception as e:
            self.get_logger().error(f"Error during disconnect: {e}")
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = MqttStatusNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()

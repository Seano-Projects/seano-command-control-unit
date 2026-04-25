#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
import ssl
import time
import os
import json
import queue
import threading
import urllib.error
import urllib.request
from datetime import datetime, timezone


class MqttStatusNode(Node):
    """
    ROS2 node untuk publish status online/offline ke MQTT broker
    dengan heartbeat realtime setiap 30 detik.
    """

    def __init__(self):
        super().__init__('mqtt_status')

        # ================= PARAMETERS =================
        self.declare_parameter('vehicle_code', 'USV-001')
        self.declare_parameter('transport.mode', 'mqtt')
        self.declare_parameter('mqtt.broker', 'mqtt.seano.cloud')
        self.declare_parameter('mqtt.port', 8883)
        self.declare_parameter('mqtt.username', 'seanomqtt')
        self.declare_parameter('mqtt.password', 'Seano2025*')
        self.declare_parameter('mqtt.keepalive', 60)
        self.declare_parameter('mqtt.status_keepalive', 5)
        self.declare_parameter('mqtt.base_topic', 'seano')
        self.declare_parameter('mqtt.heartbeat_interval', 30.0)
        self.declare_parameter('heartbeat_interval', 30.0)
        self.declare_parameter('api.base_url', 'https://api.seano.cloud')
        self.declare_parameter('api.auth.type', 'none')
        self.declare_parameter('api.auth.api_key', '')
        self.declare_parameter('api.auth.jwt', '')
        self.declare_parameter('api.timeout_sec', 5.0)
        self.declare_parameter('api.queue_size', 50)

        # Get parameters
        self.vehicle_code = self.get_parameter('vehicle_code').value
        self.transport_mode = str(self.get_parameter('transport.mode').value).strip().lower()
        if self.transport_mode not in ('mqtt', 'api', 'both'):
            self.get_logger().warn(
                f"Unknown transport.mode='{self.transport_mode}', fallback to 'mqtt'"
            )
            self.transport_mode = 'mqtt'
        self._enable_mqtt = self.transport_mode in ('mqtt', 'both')
        self._enable_api = self.transport_mode in ('api', 'both')
        self.mqtt_broker = self.get_parameter('mqtt.broker').value
        self.mqtt_port = int(self.get_parameter('mqtt.port').value)
        self.mqtt_username = self.get_parameter('mqtt.username').value
        self.mqtt_password = self.get_parameter('mqtt.password').value
        self.keepalive_default = int(self.get_parameter('mqtt.keepalive').value)
        self.keepalive = int(self.get_parameter('mqtt.status_keepalive').value)
        if self.keepalive <= 0:
            self.keepalive = self.keepalive_default
        self.base_topic = self.get_parameter('mqtt.base_topic').value
        self.heartbeat_interval = float(self.get_parameter('mqtt.heartbeat_interval').value)
        if self.heartbeat_interval <= 0.0:
            self.heartbeat_interval = float(self.get_parameter('heartbeat_interval').value)

        # Status topic
        self.status_topic = f"{self.base_topic}/{self.vehicle_code}/status"

        self.get_logger().info(f"Vehicle Code: {self.vehicle_code}")
        self.get_logger().info(f"Status Topic: {self.status_topic}")
        self.get_logger().info(f"MQTT Broker: {self.mqtt_broker}:{self.mqtt_port}")
        self.get_logger().info(
            f"Keepalive(status): {self.keepalive}s, Keepalive(default): {self.keepalive_default}s, "
            f"Heartbeat: {self.heartbeat_interval}s"
        )

        # ================= MQTT CLIENT SETUP =================
        self.client = None
        self.connected = False
        if self._enable_mqtt:
            client_id = f"{self.vehicle_code}-ros2-{os.getpid()}"

            # Cek versi paho-mqtt untuk kompatibilitas
            try:
                # Paho MQTT v2.x uses CallbackAPIVersion
                from paho.mqtt.client import CallbackAPIVersion
                self.client = mqtt.Client(
                    callback_api_version=CallbackAPIVersion.VERSION1,
                    client_id=client_id
                )
                self.get_logger().info("Using Paho MQTT v2.x (CallbackAPIVersion.VERSION1)")
            except ImportError:
                # Paho MQTT v1.x
                self.client = mqtt.Client(client_id=client_id)
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
            self.client.reconnect_delay_set(min_delay=1, max_delay=3)
            
            # Enable MQTT logging untuk debugging
            self.client.enable_logger(logger=None)  # Use Python's logging module

            # Start MQTT loop thread SEBELUM connect
            self.client.loop_start()
            self.get_logger().info("🔄 MQTT loop thread started")
            
            # Connect to broker
            self.connect_mqtt()
        else:
            self.get_logger().info('MQTT disabled (transport.mode=api)')

        # ================= API CLIENT SETUP =================
        self._api_base_url = str(self.get_parameter('api.base_url').value).rstrip('/')
        self._api_auth_type = str(self.get_parameter('api.auth.type').value).strip().lower()
        self._api_key = str(self.get_parameter('api.auth.api_key').value)
        if not self._api_key:
            self._api_key = os.getenv('SEANO_API_KEY', '')
        self._api_jwt = str(self.get_parameter('api.auth.jwt').value)
        if not self._api_jwt:
            self._api_jwt = os.getenv('SEANO_API_JWT', '')
        self._api_timeout_sec = float(self.get_parameter('api.timeout_sec').value)
        self._api_queue_size = int(self.get_parameter('api.queue_size').value)
        self._api_queue = None
        self._api_thread = None
        self._api_running = False
        self._api_auth_warned = False

        if self._enable_api and not self._api_base_url:
            self.get_logger().warn('API enabled but api.base_url is empty, disabling API')
            self._enable_api = False

        if self._enable_api:
            self._start_api_worker()
            if not self._enable_mqtt:
                self.publish_online_status(log_prefix='📤 API startup', send_api=True)

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
            self.client.connect_async(
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
            self.publish_online_status(log_prefix='📤 Initial publish')
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
            self.publish_online_status(log_prefix='💓 Heartbeat', send_api=False)
        elif not self._enable_api:
            self.get_logger().warn("⚠️  Heartbeat skipped - MQTT not connected")

    def publish_online_status(self, log_prefix='📤 Published', send_api=True):
        """Publish retained online status segera setelah konek/heartbeat."""
        if self.client is not None:
            result = self.client.publish(
                self.status_topic,
                payload="online",
                qos=1,
                retain=True
            )

            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                self.get_logger().info(f"{log_prefix}: {self.status_topic} = online")
            else:
                self.get_logger().warn(f"⚠️  Failed to publish online status (rc={result.rc})")

        if self._enable_api and send_api:
            payload = {
                'vehicle_code': self.vehicle_code,
                'status': 'online',
                'timestamp': self._now_iso_utc(),
            }
            self._api_enqueue('POST', '/vehicle-status', payload)

    def _now_iso_utc(self) -> str:
        return datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%S.%f')[:-3] + 'Z'

    def _start_api_worker(self):
        self._api_queue = queue.Queue(maxsize=max(1, self._api_queue_size))
        self._api_running = True
        self._api_thread = threading.Thread(target=self._api_worker_loop, daemon=True)
        self._api_thread.start()
        self.get_logger().info('API worker started')

    def _api_worker_loop(self):
        while self._api_running:
            try:
                item = self._api_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            if item is None:
                break

            method, endpoint, payload = item
            try:
                self._api_post_json(method, endpoint, payload)
            finally:
                self._api_queue.task_done()

    def _api_enqueue(self, method: str, endpoint: str, payload: dict):
        if not self._enable_api or self._api_queue is None:
            return
        try:
            self._api_queue.put_nowait((method, endpoint, payload))
        except queue.Full:
            self.get_logger().warn('API queue full, dropping status payload')

    def _api_post_json(self, method: str, endpoint: str, payload: dict):
        if not self._api_base_url:
            return

        headers = {
            'Content-Type': 'application/json',
            'Accept': 'application/json',
            'User-Agent': 'curl/8.5.0',
        }
        if self._api_auth_type == 'apikey':
            if self._api_key:
                headers['X-API-Key'] = self._api_key
            elif not self._api_auth_warned:
                self.get_logger().warn('API auth type apikey but api key is empty')
                self._api_auth_warned = True
        elif self._api_auth_type == 'jwt':
            if self._api_jwt:
                headers['Authorization'] = f'Bearer {self._api_jwt}'
            elif not self._api_auth_warned:
                self.get_logger().warn('API auth type jwt but token is empty')
                self._api_auth_warned = True

        url = f"{self._api_base_url}{endpoint}"
        body = json.dumps(payload, separators=(',', ':')).encode('utf-8')
        req = urllib.request.Request(url, data=body, headers=headers, method=method)

        try:
            with urllib.request.urlopen(req, timeout=self._api_timeout_sec) as resp:
                _ = resp.read()
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode('utf-8', errors='ignore')
            self.get_logger().warn(
                f"API {method} {endpoint} failed: {exc.code} {exc.reason} {detail[:200]}"
            )
        except Exception as exc:
            self.get_logger().warn(f"API {method} {endpoint} error: {exc}")

    def destroy_node(self):
        """Cleanup saat node di-shutdown"""
        self.get_logger().info("🛑 Shutting down MQTT status node...")
        
        # Publish offline status sebelum disconnect
        if self.connected and self.client is not None:
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

        if self._enable_api:
            payload = {
                'vehicle_code': self.vehicle_code,
                'status': 'offline',
                'timestamp': self._now_iso_utc(),
            }
            self._api_enqueue('POST', '/vehicle-status', payload)
            time.sleep(0.1)
        
        # Disconnect MQTT
        if self.client is not None:
            try:
                self.client.disconnect()
                self.client.loop_stop()
            except Exception as e:
                self.get_logger().error(f"Error during disconnect: {e}")

        if self._api_running:
            self._api_running = False
            if self._api_queue is not None:
                try:
                    self._api_queue.put_nowait(None)
                except queue.Full:
                    pass
            if self._api_thread is not None:
                self._api_thread.join(timeout=2.0)
        
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

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
param_node — menerima perintah set parameter via MQTT lalu mengirimkannya ke MAVROS.

MQTT topic yang disubscribe:
  seano/{vehicle_id}/mavparam

Format payload JSON:
  {
    "param_id": "CRUISE_SPEED",
    "type":     "float",          // "float"/"double", "int"/"integer", "bool", "string"
    "value":    5.0
  }

Status hasil diterbitkan ke:
  MQTT : seano/{vehicle_id}/mavparam/response
  ROS  : param_status  (std_msgs/String berisi JSON)

Service MAVROS yang dipakai:
  /mavros/param/set  (mavros_msgs/srv/ParamSetV2)
"""

import json
import ssl
from typing import Any

import paho.mqtt.client as mqtt
import rclpy
from mavros_msgs.srv import ParamSetV2
from rcl_interfaces.msg import ParameterValue
from rclpy.node import Node
from std_msgs.msg import String

# Konstanta tipe rcl_interfaces/ParameterValue
_TYPE_BOOL    = 1
_TYPE_INT     = 2
_TYPE_DOUBLE  = 3
_TYPE_STRING  = 4


def _build_param_value(type_str: str, raw_value: Any) -> ParameterValue:
    """Konversi type string + raw value ke rcl_interfaces/ParameterValue."""
    pv = ParameterValue()
    t = type_str.lower()

    if t in ('bool',):
        if isinstance(raw_value, str):
            raw_value = raw_value.lower() in ('true', '1', 'yes')
        pv.type = _TYPE_BOOL
        pv.bool_value = bool(raw_value)

    elif t in ('int', 'integer'):
        pv.type = _TYPE_INT
        pv.integer_value = int(raw_value)

    elif t in ('float', 'double', 'real'):
        pv.type = _TYPE_DOUBLE
        pv.double_value = float(raw_value)

    elif t in ('string', 'str'):
        pv.type = _TYPE_STRING
        pv.string_value = str(raw_value)

    else:
        raise ValueError(f"Tipe tidak dikenal: '{type_str}'. Gunakan float/int/bool/string.")

    return pv


class ParamNode(Node):

    def __init__(self) -> None:
        super().__init__('param_node')

        # ── Parameter ──────────────────────────────────────────────────────
        self.declare_parameter('vehicle.id', 'UNKNOWN')
        self.declare_parameter('mqtt.broker', 'mqtt.seano.cloud')
        self.declare_parameter('mqtt.port', 8883)
        self.declare_parameter('mqtt.username', '')
        self.declare_parameter('mqtt.password', '')
        self.declare_parameter('mqtt.base_topic', 'seano')
        self.declare_parameter('mqtt.qos', 1)
        self.declare_parameter('mqtt.keepalive', 60)
        self.declare_parameter('mqtt.use_tls', True)
        self.declare_parameter('mqtt.tls_insecure', False)

        # ── Ambil nilai parameter ──────────────────────────────────────────
        self._vehicle_id   = self.get_parameter('vehicle.id').value
        self._broker       = self.get_parameter('mqtt.broker').value
        self._port         = int(self.get_parameter('mqtt.port').value)
        self._username     = self.get_parameter('mqtt.username').value
        self._password     = self.get_parameter('mqtt.password').value
        self._base_topic   = self.get_parameter('mqtt.base_topic').value
        self._qos          = int(self.get_parameter('mqtt.qos').value)
        self._keepalive    = int(self.get_parameter('mqtt.keepalive').value)
        self._use_tls      = bool(self.get_parameter('mqtt.use_tls').value)
        self._tls_insecure = bool(self.get_parameter('mqtt.tls_insecure').value)

        self._mqtt_topic    = f"{self._base_topic}/{self._vehicle_id}/mavparam"
        self._resp_topic    = f"{self._mqtt_topic}/response"

        # ── ROS publisher + service client ────────────────────────────────
        self._status_pub = self.create_publisher(String, 'param_status', 10)
        self._param_set_cli = self.create_client(ParamSetV2, '/mavros/param/set')

        # ── MQTT client ────────────────────────────────────────────────────
        self._client = mqtt.Client()
        self._client.on_connect    = self._on_connect
        self._client.on_message    = self._on_message
        self._client.on_disconnect = self._on_disconnect

        if self._username:
            self._client.username_pw_set(self._username, self._password)

        if self._use_tls:
            self._client.tls_set(cert_reqs=ssl.CERT_REQUIRED)
            self._client.tls_insecure_set(self._tls_insecure)

        self._client.reconnect_delay_set(min_delay=1, max_delay=30)

        try:
            self._client.connect(self._broker, self._port, keepalive=self._keepalive)
            self._client.loop_start()
            self.get_logger().info(
                f"Param node terhubung ke MQTT {self._broker}:{self._port}"
            )
        except Exception as exc:
            self.get_logger().error(f"Gagal koneksi MQTT: {exc}")
            raise SystemExit

        self.get_logger().info(
            f"Param node aktif — vehicle: {self._vehicle_id} | "
            f"MQTT topic: {self._mqtt_topic}"
        )

    # ── MQTT callbacks ────────────────────────────────────────────────────

    def _on_connect(self, client, userdata, flags, rc, properties=None):
        if rc == 0:
            client.subscribe(self._mqtt_topic, qos=self._qos)
            self.get_logger().info(f"MQTT subscribe: {self._mqtt_topic}")
        else:
            self.get_logger().error(f"MQTT connect gagal, rc={rc}")

    def _on_disconnect(self, client, userdata, rc, properties=None):
        if rc != 0:
            self.get_logger().warn("MQTT terputus, mencoba reconnect...")

    def _on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
        except (json.JSONDecodeError, UnicodeDecodeError) as exc:
            self.get_logger().error(f"Payload tidak valid: {exc}")
            return

        if not isinstance(payload, dict):
            self.get_logger().error("Payload harus berupa JSON object")
            return

        param_id = payload.get('param_id', '').strip()
        type_str = payload.get('type', '').strip()
        raw_value = payload.get('value')

        # Validasi field wajib
        if not param_id:
            self._respond(False, None, "Field 'param_id' wajib diisi")
            return
        if not type_str:
            self._respond(False, param_id, "Field 'type' wajib diisi (float/int/bool/string)")
            return
        if raw_value is None:
            self._respond(False, param_id, "Field 'value' wajib diisi")
            return

        try:
            pv = _build_param_value(type_str, raw_value)
        except (ValueError, TypeError) as exc:
            self._respond(False, param_id, str(exc))
            return

        self.get_logger().info(
            f"Set param — {param_id} = {raw_value} (type: {type_str})"
        )

        # Kirim async ke MAVROS, sambungkan ke callback
        req = ParamSetV2.Request()
        req.param_id   = param_id
        req.value      = pv
        req.force_set  = False

        if not self._param_set_cli.wait_for_service(timeout_sec=3.0):
            self._respond(False, param_id, "Service /mavros/param/set tidak tersedia")
            return

        future = self._param_set_cli.call_async(req)
        future.add_done_callback(
            lambda f, pid=param_id: self._on_param_set_done(f, pid)
        )

    # ── Service callback ──────────────────────────────────────────────────

    def _on_param_set_done(self, future, param_id: str):
        try:
            result = future.result()
        except Exception as exc:
            self._respond(False, param_id, f"Service call error: {exc}")
            return

        if result.success:
            self.get_logger().info(f"Param '{param_id}' berhasil di-set")
            self._respond(True, param_id, "OK")
        else:
            self.get_logger().warn(f"Param '{param_id}' gagal di-set oleh FCU")
            self._respond(False, param_id, "FCU menolak parameter")

    # ── Response helpers ──────────────────────────────────────────────────

    def _respond(self, success: bool, param_id, message: str):
        body = {
            'success':  success,
            'param_id': param_id,
            'message':  message,
        }
        raw = json.dumps(body)

        # Publish ke ROS
        ros_msg = String()
        ros_msg.data = raw
        self._status_pub.publish(ros_msg)

        # Publish ke MQTT
        self._client.publish(self._resp_topic, raw, qos=self._qos)
        self.get_logger().info(f"Response param: {raw}")

    # ── Cleanup ────────────────────────────────────────────────────────────

    def destroy_node(self) -> None:
        self._client.loop_stop()
        self._client.disconnect()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ParamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

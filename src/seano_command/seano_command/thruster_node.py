#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
thruster_node — menerima perintah thruster dari MQTT lalu publish ke /mavros/rc/override.

MQTT topic yang disubscribe:
  seano/{vehicle_id}/thruster

Format payload JSON:
  {
    "throttle": 50,    // -100..100  (negatif = mundur)
    "steering": -30    // -100..100  (negatif = kiri, positif = kanan)
  }

Nilai 0 pada throttle maupun steering berarti netral (PWM 1500 µs).
Untuk melepas override (biarkan RC fisik yang kontrol), kirim:
  {"release": true}

Mapping channel ArduRover default:
  CH1 (index 0) = Steering
  CH3 (index 2) = Throttle
"""

import json
import ssl

import paho.mqtt.client as mqtt
import rclpy
from mavros_msgs.msg import OverrideRCIn
from rclpy.node import Node


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def _map_to_pwm(value: float, neutral: int, min_pwm: int, max_pwm: int) -> int:
    """Map nilai -100..100 ke rentang PWM µs."""
    if value >= 0.0:
        pwm = neutral + (value / 100.0) * (max_pwm - neutral)
    else:
        pwm = neutral + (value / 100.0) * (neutral - min_pwm)
    return int(round(_clamp(pwm, min_pwm, max_pwm)))


class ThrusterNode(Node):

    CHAN_RELEASE = 0  # nilai 0 = lepas override channel ini ke ArduPilot

    def __init__(self) -> None:
        super().__init__('thruster_node')

        # ── Parameter ──────────────────────────────────────────────────────
        self.declare_parameter('vehicle.id', 'UNKNOWN')
        self.declare_parameter('mqtt.broker', 'localhost')
        self.declare_parameter('mqtt.port', 1883)
        self.declare_parameter('mqtt.username', '')
        self.declare_parameter('mqtt.password', '')
        self.declare_parameter('mqtt.base_topic', 'seano')
        self.declare_parameter('mqtt.qos', 1)
        self.declare_parameter('mqtt.keepalive', 60)
        self.declare_parameter('mqtt.use_tls', True)
        self.declare_parameter('mqtt.tls_insecure', True)

        self.declare_parameter('thruster.pwm_neutral', 1500)
        self.declare_parameter('thruster.pwm_min', 1000)
        self.declare_parameter('thruster.pwm_max', 2000)
        self.declare_parameter('thruster.channel_throttle', 2)   # CH3 (0-indexed)
        self.declare_parameter('thruster.channel_steering', 0)   # CH1 (0-indexed)
        self.declare_parameter('thruster.allow_reverse', True)   # izinkan mundur

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

        self._pwm_neutral      = int(self.get_parameter('thruster.pwm_neutral').value)
        self._pwm_min          = int(self.get_parameter('thruster.pwm_min').value)
        self._pwm_max          = int(self.get_parameter('thruster.pwm_max').value)
        self._ch_throttle      = int(self.get_parameter('thruster.channel_throttle').value)
        self._ch_steering      = int(self.get_parameter('thruster.channel_steering').value)
        self._allow_reverse    = bool(self.get_parameter('thruster.allow_reverse').value)

        self._mqtt_topic = f"{self._base_topic}/{self._vehicle_id}/thruster"

        # ── State terakhir untuk resend periodik ──────────────────────────
        self._last_pwm_thr: int = self._pwm_neutral
        self._last_pwm_str: int = self._pwm_neutral
        self._override_active: bool = False  # True = ada perintah aktif

        # ── ROS publisher + timer resend 10 Hz ────────────────────────────
        self._rc_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        self._resend_timer = self.create_timer(0.1, self._resend_override)  # 10 Hz

        # ── MQTT client ────────────────────────────────────────────────────
        self._client = mqtt.Client()
        self._client.on_connect    = self._on_connect
        self._client.on_message    = self._on_message
        self._client.on_disconnect = self._on_disconnect

        if self._username:
            self._client.username_pw_set(self._username, self._password)

        if self._use_tls:
            self._client.tls_set(cert_reqs=ssl.CERT_NONE)
            self._client.tls_insecure_set(self._tls_insecure)

        self._client.reconnect_delay_set(min_delay=1, max_delay=30)

        try:
            self._client.loop_start()
            # connect_async keeps node alive when DNS/network is not ready at boot.
            self._client.connect_async(self._broker, self._port, keepalive=self._keepalive)
            self.get_logger().info(
                f"Thruster MQTT connect scheduled: {self._broker}:{self._port}"
            )
        except Exception as exc:
            self.get_logger().error(
                f"Gagal startup MQTT (node tetap jalan, retry otomatis): {exc}"
            )

        self.get_logger().info(
            f"Thruster node aktif — vehicle: {self._vehicle_id} | "
            f"MQTT topic: {self._mqtt_topic} | "
            f"CH throttle: {self._ch_throttle + 1} | "
            f"CH steering: {self._ch_steering + 1}"
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

        # Lepas override jika ada key "release": true
        if payload.get('release', False):
            self._override_active = False
            self._publish_release()
            return

        throttle_raw = payload.get('throttle', 0.0)
        steering_raw = payload.get('steering', 0.0)

        try:
            throttle = float(throttle_raw)
            steering = float(steering_raw)
        except (TypeError, ValueError):
            self.get_logger().error(
                f"Nilai throttle/steering harus numerik: {payload}"
            )
            return

        # Kalau mundur tidak diizinkan, clamp throttle ke 0..100
        if not self._allow_reverse:
            throttle = _clamp(throttle, 0.0, 100.0)

        throttle = _clamp(throttle, -100.0, 100.0)
        steering = _clamp(steering, -100.0, 100.0)

        pwm_thr = _map_to_pwm(throttle, self._pwm_neutral, self._pwm_min, self._pwm_max)
        # Negate steering: rc1 HIGH=kiri, rc1 LOW=kanan.
        # Konvensi API: negatif=kiri, positif=kanan → perlu dibalik sebelum di-map.
        pwm_str = _map_to_pwm(-steering, self._pwm_neutral, self._pwm_min, self._pwm_max)

        self.get_logger().info(
            f"Thruster — throttle: {throttle:+.0f}% → {pwm_thr}µs | "
            f"steering: {steering:+.0f}% → {pwm_str}µs"
        )

        self._last_pwm_thr = pwm_thr
        self._last_pwm_str = pwm_str
        self._override_active = True
        self._publish_override(pwm_thr, pwm_str)

    # ── Publish helpers ───────────────────────────────────────────────────

    def _resend_override(self) -> None:
        """Timer callback 10 Hz — kirim ulang nilai terakhir agar tidak timeout."""
        if self._override_active:
            self._publish_override(self._last_pwm_thr, self._last_pwm_str)

    def _publish_override(self, pwm_thr: int, pwm_str: int) -> None:
        msg = OverrideRCIn()
        # Semua channel diset CHAN_RELEASE (tidak override) kecuali yang dipakai
        msg.channels = [OverrideRCIn.CHAN_NOCHANGE] * 18

        msg.channels[self._ch_throttle] = pwm_thr
        msg.channels[self._ch_steering] = pwm_str

        self._rc_pub.publish(msg)

    def _publish_release(self) -> None:
        msg = OverrideRCIn()
        msg.channels = [self.CHAN_RELEASE] * 18
        self._rc_pub.publish(msg)
        self.get_logger().info("Override dilepas — RC fisik aktif kembali")

    def destroy_node(self) -> None:
        self._client.loop_stop()
        self._client.disconnect()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ThrusterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

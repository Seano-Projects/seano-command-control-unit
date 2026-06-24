import csv
import json
import os
import re
import shutil
import subprocess
import time
import urllib.error
import urllib.request
import urllib.parse
from datetime import datetime
from zoneinfo import ZoneInfo

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

_TZ = ZoneInfo('Asia/Jakarta')


def _now_iso() -> str:
    return datetime.now(_TZ).strftime('%Y-%m-%dT%H:%M:%S.%f')[:-3]


class _DailyCsvWriter:
    """CSV writer with daily rotation (YYYYMMDD)."""

    def __init__(self, log_dir: str, prefix: str, fieldnames: list):
        self._log_dir = log_dir
        self._prefix = prefix
        self._fieldnames = fieldnames
        self._date = None
        self._fh = None
        self._writer = None
        os.makedirs(log_dir, exist_ok=True)

    def _rotate(self):
        today = datetime.now().strftime('%Y%m%d')
        if today == self._date:
            return
        if self._fh:
            self._fh.close()
        self._date = today
        path = os.path.join(self._log_dir, f'{self._prefix}_{today}.csv')
        first = not os.path.exists(path)
        self._fh = open(path, 'a', newline='', encoding='utf-8')
        self._writer = csv.DictWriter(self._fh, fieldnames=self._fieldnames, extrasaction='ignore')
        if first:
            self._writer.writeheader()

    def write(self, row: dict):
        self._rotate()
        self._writer.writerow(row)
        self._fh.flush()

    def close(self):
        if self._fh:
            self._fh.close()
            self._fh = None


_FIELDS = [
    'timestamp',
    'vehicle_code',
    'url',
    'method',
    'ok',
    'http_status',
    'latency_ms',
    'icmp_ok',
    'icmp_rtt_ms',
    'error',
]


class NetworkMonitorNode(Node):
    def __init__(self):
        super().__init__('network_monitor')

        self.declare_parameter('vehicle.id', 'USV-001')
        self.declare_parameter('monitor.url', 'https://seano.cloud/')
        self.declare_parameter('monitor.interval_sec', 5.0)
        self.declare_parameter('monitor.timeout_sec', 3.0)
        self.declare_parameter('monitor.method', 'HEAD')
        self.declare_parameter('monitor.publish_topic', '')
        self.declare_parameter('monitor.user_agent', 'SeanoNetMon/1.0')
        self.declare_parameter('icmp.enabled', True)
        self.declare_parameter('icmp.host', 'seano.cloud')
        self.declare_parameter('icmp.count', 1)
        self.declare_parameter('icmp.timeout_sec', 1.0)
        self.declare_parameter('logger.log_dir', '~/Seano_ws/ros_log')
        self.declare_parameter('logger.max_error_len', 160)

        self.vehicle_id = str(self.get_parameter('vehicle.id').value)
        self.url = str(self.get_parameter('monitor.url').value).strip() or 'https://seano.cloud/'
        self.interval_sec = max(0.5, float(self.get_parameter('monitor.interval_sec').value))
        self.timeout_sec = max(0.5, float(self.get_parameter('monitor.timeout_sec').value))
        self.method = str(self.get_parameter('monitor.method').value).strip().upper() or 'HEAD'
        self.publish_topic = str(self.get_parameter('monitor.publish_topic').value).strip()
        self.user_agent = str(self.get_parameter('monitor.user_agent').value).strip() or 'SeanoNetMon/1.0'
        self.icmp_enabled = bool(self.get_parameter('icmp.enabled').value)
        self.icmp_host = str(self.get_parameter('icmp.host').value).strip()
        self.icmp_count = max(1, int(self.get_parameter('icmp.count').value))
        self.icmp_timeout_sec = max(
            0.5,
            float(self.get_parameter('icmp.timeout_sec').value),
        )
        self.max_error_len = int(self.get_parameter('logger.max_error_len').value)

        log_dir = os.path.expanduser(self.get_parameter('logger.log_dir').value)
        self._csv = _DailyCsvWriter(
            os.path.join(log_dir, 'network'),
            'network_log',
            _FIELDS,
        )

        self._publisher = None
        if self.publish_topic:
            self._publisher = self.create_publisher(String, self.publish_topic, 10)
        self._timer = self.create_timer(self.interval_sec, self._tick)

        self.get_logger().info('Network monitor started')
        self.get_logger().info(f'URL: {self.url}')
        self.get_logger().info(f'Interval: {self.interval_sec}s, Timeout: {self.timeout_sec}s')

    def _tick(self):
        ts = _now_iso()
        ok, status, latency_ms, error, method_used = self._ping_once()
        icmp_ok, icmp_rtt = self._icmp_ping()

        payload = {
            'timestamp': ts,
            'vehicle_code': self.vehicle_id,
            'url': self.url,
            'method': method_used,
            'ok': ok,
            'http_status': status,
            'latency_ms': latency_ms,
            'icmp_ok': icmp_ok,
            'icmp_rtt_ms': self._format_rtt_ms(icmp_rtt),
            'error': error,
        }

        self._csv.write(payload)

        if self._publisher is not None:
            msg = String()
            msg.data = json.dumps(payload, separators=(',', ':'))
            self._publisher.publish(msg)

    def _ping_once(self):
        start = time.monotonic()
        status = None
        error = ''
        method_used = self.method

        try:
            status = self._request(self.method)
        except urllib.error.HTTPError as exc:
            status = exc.code
            if self.method == 'HEAD' and status in (405, 501):
                try:
                    method_used = 'GET'
                    status = self._request('GET')
                except Exception as exc2:
                    error = f'GET failed: {exc2}'
            else:
                error = f'HTTPError {status}: {exc.reason}'
        except urllib.error.URLError as exc:
            error = f'URLError: {exc.reason}'
        except Exception as exc:
            error = f'{type(exc).__name__}: {exc}'

        latency_ms = int((time.monotonic() - start) * 1000)
        ok = status is not None and 200 <= int(status) < 400
        if not ok and not error and status is not None:
            error = f'HTTP {status}'

        if error and self.max_error_len > 0:
            error = str(error)[: self.max_error_len]

        return ok, status, latency_ms, error, method_used

    def _request(self, method: str) -> int:
        headers = {'User-Agent': self.user_agent}
        req = urllib.request.Request(self.url, method=method, headers=headers)
        with urllib.request.urlopen(req, timeout=self.timeout_sec) as resp:
            status = resp.getcode()
            if method == 'GET':
                resp.read(1)
            return int(status)

    def _icmp_ping(self):
        if not self.icmp_enabled:
            return None, None
        if not shutil.which('ping'):
            return None, None

        host = self.icmp_host or self._host_from_url(self.url)
        if not host:
            return None, None

        timeout_sec = max(1, int(self.icmp_timeout_sec))
        args = ['ping', '-c', str(self.icmp_count), '-W', str(timeout_sec), host]
        output, rc = self._run_cmd_with_rc(args)
        if rc != 0:
            return False, None

        rtt = self._parse_ping_rtt(output)
        if rtt is None:
            return False, None
        return True, rtt

    def _parse_ping_rtt(self, output: str):
        if not output:
            return None
        match = re.search(r'time=([0-9.]+)\s*ms', output)
        if match:
            return float(match.group(1))
        return None

    def _host_from_url(self, url: str) -> str:
        try:
            parsed = urllib.parse.urlparse(url)
        except Exception:
            return ''
        return parsed.hostname or ''

    def _format_rtt_ms(self, value):
        if value is None:
            return ''
        return round(float(value), 1)

    def _run_cmd_with_rc(self, args):
        try:
            result = subprocess.run(
                args,
                capture_output=True,
                text=True,
                timeout=max(1.0, float(self.icmp_timeout_sec) + 1.0),
                check=False,
            )
        except Exception:
            return '', 1
        return (result.stdout or ''), int(result.returncode)

    def destroy_node(self):
        self._csv.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = NetworkMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

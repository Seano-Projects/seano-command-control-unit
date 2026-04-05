# seano_mission

Package ini memonitor status misi (mission) USV melalui MAVROS: waypoint yang sedang aktif, waypoint yang sudah dicapai, home position, dan status koneksi.

## Node

- `mission_node`: subscribe topic MAVROS terkait mission, publish status misi ke ROS2 topic.

## ROS2 Interface

### Subscribed (dari MAVROS)

| Topic | Tipe | Keterangan |
|---|---|---|
| `/mavros/state` | `mavros_msgs/State` | Status koneksi, armed, mode |
| `/mavros/mission/reached` | `mavros_msgs/WaypointReached` | Event waypoint dicapai |
| `/mavros/mission/waypoints` | `mavros_msgs/WaypointList` | Daftar waypoint aktif |
| `/mavros/home_position/home` | `mavros_msgs/HomePosition` | Koordinat home |

### Published

| Topic | Tipe | Keterangan |
|---|---|---|
| `/seano/mission/status` | `std_msgs/String` (JSON) | Status misi lengkap, publish tiap 2 detik |
| `/seano/mission/waypoint_reached` | `std_msgs/String` (JSON) | Event saat waypoint dicapai |

### Contoh Payload `/seano/mission/status`

```json
{
  "vehicle_id": "USV-001",
  "connected": true,
  "armed": true,
  "mode": "AUTO",
  "mission_active": true,
  "current_wp_seq": 2,
  "total_waypoints": 5,
  "last_reached_seq": 1,
  "remaining_waypoints": 2,
  "home": { "lat": -6.123456, "lon": 106.123456, "alt": 10.5 },
  "current_waypoint": {
    "seq": 2,
    "lat": -6.124000,
    "lon": 106.124000,
    "alt": 0.0,
    "param1": 2.0
  }
}
```

### Contoh Payload `/seano/mission/waypoint_reached`

```json
{
  "vehicle_id": "USV-001",
  "event": "waypoint_reached",
  "wp_seq": 1,
  "total": 5,
  "remaining": 3
}
```

> **Catatan**: `param1` di `current_waypoint` untuk command `NAV_WAYPOINT` (cmd=16) adalah acceptance radius dalam meter (0 = pakai default ArduPilot `WP_RADIUS`).

## MQTT

Tidak ada koneksi MQTT langsung. Data bisa diforward oleh `seano_mqtt_bridge` dari topic `/seano/mission/status`.

## Cara Build dan Jalankan

```bash
cd ~/Seano_ws
colcon build --packages-select seano_mission
source install/setup.bash

ros2 run seano_mission mission_node --ros-args --params-file ~/Seano_ws/src/seano_startup/config/system.yaml
```

## Verifikasi

```bash
# Cek topic mission status
ros2 topic echo /seano/mission/status

# Cek event waypoint reached
ros2 topic echo /seano/mission/waypoint_reached
```

# seano_logging

Package ini adalah kombinasi logger lama (`telemetry_logger_node`) dan logger baru dari project `seano-logger` (`logger_node`).

## Node

- `logger_node`: auto-detect sensor topic, lalu simpan log `.log` dan `.csv` per sensor ke folder misi.
- `telemetry_logger_node`: subscribe telemetry JSON dan tulis ke CSV.

## ROS2 Interface

### Published

- Tidak ada publisher ROS2.

### Subscribed - `logger_node`

- `/mavros/global_position/global` (`sensor_msgs/NavSatFix`)
- `/mavros/imu/data` (`sensor_msgs/Imu`)
- `/mavros/battery` (`sensor_msgs/BatteryState`)
- `oceanography/ctd` (`std_msgs/String`, JSON)
- `oceanography/adcp` (`std_msgs/String`, JSON)
- `oceanography/sbes` (`std_msgs/String`, JSON)
- `telemetry` (`std_msgs/String`, JSON)

### Subscribed - `telemetry_logger_node`

- `telemetry` (`std_msgs/String`) berisi JSON telemetry

## MQTT

Tidak ada koneksi MQTT langsung di package ini.

## Parameter Penting - `telemetry_logger_node`

- `vehicle.id`
- `logging.enable`
- `logging.path`
- `logging.format` (saat ini implementasi utama CSV)

## Parameter Penting - `logger_node`

- `logging.mount_point` (lokasi mount SSD eksternal)
- `logging.flush_interval`
- `logging.auto_mount` (jika `true`, node akan coba mount SSD otomatis saat startup)
- `logging.device_uuid` (disarankan diisi agar deteksi device lebih akurat)
- `logging.device_label` (fallback kalau UUID belum diisi)
- `logging.topics.gps`, `logging.topics.imu`, `logging.topics.battery`
- `logging.topics.ctd`, `logging.topics.adcp`, `logging.topics.sbes`, `logging.topics.telemetry`

## Output File

### `logger_node`

- Path default: `/media/seano/SEANO_SSD/SEANO_MISSIONS/YYYY/MM/DD/MISSION_START_HH-MM-SS_TZ/`
- Generate file per sensor: `gps.log/csv`, `imu.log/csv`, `ctd.log/csv`, `adcp.log/csv`, `battery.log/csv`
- Tambahan file: `sbes.log/csv`, `telemetry.log/csv` (akan dibuat jika topic terdeteksi)

Catatan auto-mount:
- Logger akan cek mount point saat startup.
- Jika belum ter-mount dan `logging.auto_mount=true`, node akan mount otomatis pakai `udisksctl`.
- Pastikan paket `udisks2` tersedia di Ubuntu.

### `telemetry_logger_node`

- Format nama file: `{vehicle_id}_telemetry_YYYYMMDD_HHMMSS.csv`
- Lokasi file mengikuti parameter `logging.path`

## Jalankan

```bash
ros2 run seano_logging logger_node

ros2 run seano_logging telemetry_logger_node --ros-args --params-file /home/seano/Seano_ws/src/seano_startup/config/system.yaml
```

## Mode Menjalankan (Fleksibel)

### 1) Jalan bareng semua package

`seano_logging` otomatis ikut jalan saat:

```bash
ros2 launch seano_startup system.launch.py
```

### 2) Jalan satu-satu (untuk debug)

```bash
ros2 run seano_logging logger_node --ros-args --params-file /home/seano/Seano_ws/src/seano_startup/config/system.yaml
ros2 run seano_logging telemetry_logger_node --ros-args --params-file /home/seano/Seano_ws/src/seano_startup/config/system.yaml
```

### Cek cepat

```bash
ros2 node list | grep logger
```

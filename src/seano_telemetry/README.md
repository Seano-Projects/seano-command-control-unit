# seano_telemetry

Package ini mengumpulkan data MAVROS (state, GPS, IMU, battery, speed, radio, temperature) lalu menggabungkannya menjadi telemetry JSON untuk dipakai package lain.

## Node

- `telemetry_node`: subscribe beberapa topic MAVROS, olah data, lalu publish telemetry JSON periodik.

## ROS2 Interface

### Published

- `telemetry` (`std_msgs/String`, JSON) dipublish setiap 1 detik

Field utama payload telemetry:
- `vehicle_code`
- `battery_voltage`, `battery_current`, `battery_percentage`
- `rssi`
- `latitude`, `longitude`, `altitude`
- `heading`, `roll`, `pitch`, `yaw`
- `armed`, `gps_ok`, `system_status`, `mode`, `speed`
- `temperature_system`

### Subscribed

- `/mavros/state` (`mavros_msgs/State`)
- `/mavros/global_position/global` (`sensor_msgs/NavSatFix`)
- `/mavros/imu/data` (`sensor_msgs/Imu`)
- `/mavros/battery` (`sensor_msgs/BatteryState`)
- `/mavros/vfr_hud` (`mavros_msgs/VfrHud`)
- `/mavros/radio_status` (`mavros_msgs/RadioStatus`)
- `/mavros/temperature` (`sensor_msgs/Temperature`)

## MQTT

Tidak ada koneksi MQTT langsung di package ini.

Catatan: data dari topic `telemetry` biasanya diteruskan ke MQTT oleh package `seano_mqtt_bridge`.

## Parameter Penting

- `system.mode`
- `vehicle.id`

## Jalankan

```bash
ros2 run seano_telemetry telemetry_node --ros-args --params-file /home/seano/Seano_ws/src/seano_startup/config/system.yaml
```

## Mode Menjalankan (Fleksibel)

### 1) Jalan bareng semua package

`seano_telemetry` otomatis ikut jalan saat:

```bash
ros2 launch seano_startup system.launch.py
```

### 2) Jalan satu-satu (untuk debug)

```bash
ros2 run seano_telemetry telemetry_node --ros-args --params-file /home/seano/Seano_ws/src/seano_startup/config/system.yaml
```

### Cek cepat

```bash
ros2 node list | grep telemetry
ros2 topic echo /usv/telemetry
```

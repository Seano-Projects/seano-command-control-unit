# seano_anti_theft

Package ini memantau data MAVROS untuk deteksi anti-theft seperti geofence breach, towing, dan indikasi boat flipped.

## Node

- `anti_theft_node`: subscribe topic MAVROS, evaluasi rule keamanan, publish telemetry dan alert, kirim telemetry anti-theft ke MQTT.

## ROS2 Interface

### Published

- `anti_theft/telemetry_json` (`std_msgs/String`, JSON)
- `anti_theft/alert` (`std_msgs/String`)

### Subscribed

- `/mavros/global_position/global` (`sensor_msgs/NavSatFix`)
- `/mavros/imu/data` (`sensor_msgs/Imu`)
- `/mavros/vfr_hud` (`mavros_msgs/VfrHud`)
- `/mavros/state` (`mavros_msgs/State`)
- `/mavros/rc/in` (`mavros_msgs/RCIn`)

## MQTT

### Kirim (Publish)

- `seano/{vehicle_id}/anti_theft/telemetry`
	- Payload: JSON anti-theft (lat/lon/tilt/drift/mode/alarm/dll)

### Terima (Subscribe)

- Tidak ada subscribe MQTT.

## Parameter Penting

- `vehicle.id`
- `anti_theft.loop_rate_hz`
- `anti_theft.target_speed_mps`
- `anti_theft.mavros.gps_topic`
- `anti_theft.mavros.imu_topic`
- `anti_theft.mavros.vfr_topic`
- `anti_theft.mavros.state_topic`
- `anti_theft.mavros.rc_in_topic`
- `anti_theft.mavros.set_mode_service`
- `anti_theft.geofence_limit`
- `anti_theft.crit_tilt_deg`
- `anti_theft.tilt_confirm_time`
- `anti_theft.rc_failsafe_pwm`
- `anti_theft.mission_speed_margin`
- `anti_theft.mqtt_enabled`
- `mqtt.broker`, `mqtt.port`, `mqtt.username`, `mqtt.password`, `mqtt.base_topic`, `mqtt.qos`, `mqtt.keepalive`, `mqtt.use_tls`

## Jalankan

```bash
ros2 run seano_anti_theft anti_theft_node --ros-args --params-file /home/seano/Seano_ws/src/seano_startup/config/system.yaml
```

Atau via launch package:

```bash
ros2 launch seano_anti_theft anti_theft.launch.py
```

## Mode Menjalankan (Fleksibel)

### 1) Jalan bareng semua package

`seano_anti_theft` otomatis ikut jalan saat:

```bash
ros2 launch seano_startup system.launch.py
```

### 2) Jalan satu-satu (untuk debug)

```bash
ros2 run seano_anti_theft anti_theft_node --ros-args --params-file /home/seano/Seano_ws/src/seano_startup/config/system.yaml
```

### Cek cepat

```bash
ros2 node list | grep anti_theft
ros2 topic echo /usv/anti_theft/alert
```

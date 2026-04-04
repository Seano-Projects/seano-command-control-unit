# seano_oceanography

Package sensor oseanografi untuk data CTD, ADCP, dan SBES.

## Node

- `ctd_sensor_node`: publish data CTD ke ROS2 dan MQTT.
- `adcp_sensor_node`: publish data ADCP simulasi ke ROS2.
- `sbes_sensor_node`: publish data SBES simulasi ke ROS2.

## ROS2 Interface

### Published

- `oceanography/ctd` (`std_msgs/String`, JSON) oleh `ctd_sensor_node` (default, bisa diubah parameter)
- `oceanography/adcp` (`std_msgs/String`, JSON) oleh `adcp_sensor_node` (default)
- `oceanography/sbes` (`std_msgs/String`, JSON) oleh `sbes_sensor_node` (default)

### Subscribed

- `/mavros/global_position/global` (`sensor_msgs/NavSatFix`) oleh `ctd_sensor_node`

## MQTT

### Kirim (Publish)

- `seano/{vehicle_code}/{sensor_code}/data` oleh `ctd_sensor_node`
- Payload: JSON CTD (timestamp, posisi, depth, temperature, conductivity, salinity, density, sound velocity, dll)

### Terima (Subscribe)

- Tidak ada subscribe MQTT di package ini.

## Parameter Penting

- `oceanography.ctd.publish_topic`
- `oceanography.ctd.publish_rate_hz`
- `oceanography.ctd.sensor_code`
- `oceanography.ctd.gps_topic`
- `oceanography.ctd.default_latitude`, `oceanography.ctd.default_longitude`, `oceanography.ctd.default_altitude`
- `oceanography.ctd.max_depth_m`, `oceanography.ctd.cycle_seconds`
- `mqtt.broker`, `mqtt.port`, `mqtt.username`, `mqtt.password`, `mqtt.base_topic`, `mqtt.qos`

## Jalankan

```bash
ros2 run seano_oceanography ctd_sensor_node --ros-args --params-file /home/seano/Seano_ws/src/seano_startup/config/system.yaml
ros2 run seano_oceanography adcp_sensor_node
ros2 run seano_oceanography sbes_sensor_node
```

## Mode Menjalankan (Fleksibel)

### 1) Jalan bareng semua package

`seano_oceanography` otomatis ikut jalan saat:

```bash
ros2 launch seano_startup system.launch.py
```

Catatan: default startup menjalankan `ctd_sensor_node`.

### 2) Jalan satu-satu (untuk debug)

```bash
ros2 run seano_oceanography ctd_sensor_node --ros-args --params-file /home/seano/Seano_ws/src/seano_startup/config/system.yaml
ros2 run seano_oceanography adcp_sensor_node
ros2 run seano_oceanography sbes_sensor_node
```

### Cek cepat

```bash
ros2 node list | grep ctd
ros2 topic echo /usv/oceanography/ctd
```

# seano_mqtt_bridge

Package ini menjembatani data ROS2 ke MQTT dan mengirim status online/offline USV ke broker MQTT.

## Node

- `mqtt_bridge_node`: subscribe telemetry ROS2 lalu publish ke MQTT.
- `mqtt_status_node`: publish status online/offline + heartbeat ke MQTT (retain + LWT).

## ROS2 Interface

### Published

- Tidak ada publisher ROS2 di package ini.

### Subscribed

- `telemetry` (`std_msgs/String`, JSON) oleh `mqtt_bridge_node`

## MQTT

### Kirim (Publish)

- `seano/{vehicle_id}/telemetry` oleh `mqtt_bridge_node`
  - Payload: string JSON dari topic ROS `telemetry`
- `seano/{vehicle_code}/status` oleh `mqtt_status_node`
  - Payload: `online` (periodik heartbeat)
  - Payload: `offline` saat shutdown/disconnect (LWT)
  - `retain=true`, sehingga status terakhir tersimpan di broker

### Terima (Subscribe)

- Tidak ada subscribe MQTT pada implementasi saat ini.

## Parameter Penting

### `mqtt_bridge_node`

- `vehicle.id`
- `mqtt.broker`, `mqtt.port`, `mqtt.username`, `mqtt.password`
- `mqtt.base_topic`, `mqtt.qos`

### `mqtt_status_node`

- `vehicle_code`
- `mqtt.broker`, `mqtt.port`, `mqtt.username`, `mqtt.password`
- `mqtt.keepalive`, `mqtt.base_topic`
- `heartbeat_interval`

## Jalankan

```bash
ros2 run seano_mqtt_bridge mqtt_bridge_node --ros-args --params-file /home/seano/Seano_ws/src/seano_startup/config/system.yaml
ros2 run seano_mqtt_bridge mqtt_status_node --ros-args --params-file /home/seano/Seano_ws/src/seano_mqtt_bridge/config/mqtt_status.yaml
```

Atau untuk status node via launch:

```bash
ros2 launch seano_mqtt_bridge mqtt_status.launch.py
```

## Mode Menjalankan (Fleksibel)

### 1) Jalan bareng semua package

`seano_mqtt_bridge` otomatis ikut jalan saat:

```bash
ros2 launch seano_startup system.launch.py
```

### 2) Jalan satu-satu (untuk debug)

```bash
ros2 run seano_mqtt_bridge mqtt_bridge_node --ros-args --params-file /home/seano/Seano_ws/src/seano_startup/config/system.yaml
ros2 run seano_mqtt_bridge mqtt_status_node --ros-args --params-file /home/seano/Seano_ws/src/seano_startup/config/system.yaml
```

### Cek cepat

```bash
ros2 node list | grep mqtt
ros2 topic echo /usv/telemetry
```

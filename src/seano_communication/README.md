# seano_communication

Package ini melakukan monitoring kualitas jaringan (GSM, WiFi, Ethernet) dan mengganti default route ke interface terbaik.

## Node

- `communication_node`: cek latency dan bandwidth tiap interface lalu switch route otomatis.

## ROS2 Interface

### Published

- `communication/status` (`std_msgs/String`)
  - Contoh nilai: `SWITCHED_TO:wwan0`, `SWITCHED_TO:wlP1p1s0`

### Subscribed

- Tidak ada subscriber ROS2 di node ini.

## MQTT

Tidak ada koneksi MQTT langsung di package ini.

## Parameter Penting

- `communication.gsm_interface`
- `communication.wifi_interface`
- `communication.ethernet_interface`
- `communication.ping_target`
- `communication.speed_test_url`
- `communication.latency_threshold`
- `communication.bandwidth_threshold`
- `communication.check_interval`

## Jalankan

```bash
ros2 run seano_communication communication_node --ros-args --params-file /home/seano/Seano_ws/src/seano_startup/config/system.yaml
```

## Mode Menjalankan (Fleksibel)

### 1) Jalan bareng semua package

`seano_communication` otomatis ikut jalan saat:

```bash
ros2 launch seano_startup system.launch.py
```

### 2) Jalan satu-satu (untuk debug)

```bash
ros2 run seano_communication communication_node --ros-args --params-file /home/seano/Seano_ws/src/seano_startup/config/system.yaml
```

### Cek cepat

```bash
ros2 node list | grep communication
ros2 topic echo /usv/communication/status
```

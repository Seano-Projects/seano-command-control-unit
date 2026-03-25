# README ROS - Cara Menjalankan Tiap Node

Dokumen ini berisi cara build workspace dan menjalankan node ROS2 di project SEANO.

## 1) Build Workspace

Jalankan dari root workspace (`Seano_ws`):

```bash
colcon build
```

Kalau mau build package tertentu saja:

```bash
colcon build --packages-select seano_cam seano_command seano_communication seano_failsafe seano_logging seano_mqtt_bridge seano_oceanography seano_startup seano_telemetry
```

## 2) Source Environment

Setelah build selesai, source environment:

```bash
source install/setup.bash
```

## 3) Jalankan Tiap Node

Format umum:

```bash
ros2 run <nama_package> <nama_executable>
```

### Package `seano_cam`

```bash
ros2 run seano_cam camera_node
ros2 run seano_cam camera_viewer
ros2 run seano_cam rtmp_streamer
```

### Package `seano_command`

```bash
ros2 run seano_command command_node
```

### Package `seano_communication`

```bash
ros2 run seano_communication communication_node
```

### Package `seano_failsafe`

```bash
ros2 run seano_failsafe seano_battery
ros2 run seano_failsafe seano_failsafe
ros2 run seano_failsafe seano_communication_monitor
```

### Package `seano_logging`

```bash
ros2 run seano_logging telemetry_logger_node
```

### Package `seano_mqtt_bridge`

```bash
ros2 run seano_mqtt_bridge mqtt_bridge_node
ros2 run seano_mqtt_bridge mqtt_status_node
```

### Package `seano_oceanography`

```bash
# CTD generator dengan parameter dari system.yaml
ros2 run seano_oceanography ctd_sensor_node --ros-args --params-file src/seano_startup/config/system.yaml

# Node lain (dummy/simulasi)
ros2 run seano_oceanography adcp_sensor_node
ros2 run seano_oceanography sbes_sensor_node
```

### Package `seano_telemetry`

```bash
ros2 run seano_telemetry telemetry_node
```

## 4) Menjalankan via Launch File

Selain `ros2 run`, beberapa package menyediakan launch file.

### Launch di `seano_startup`

```bash
ros2 launch seano_startup system.launch.py
```

### Launch di `seano_mqtt_bridge`

```bash
ros2 launch seano_mqtt_bridge mqtt_status.launch.py
```

## 5) Monitoring Data CTD

Monitor data CTD dari ROS topic:

```bash
ros2 topic echo /oceanography/ctd
```

Monitor data CTD di MQTT:

```bash
mosquitto_sub -h mqtt.seano.cloud -p 8883 -u seanomqtt -P 'Seano2025*' --insecure -t 'seano/USV-001/CTD-MIDAS-3000/data'
```

Format topic MQTT CTD:

```text
seano/{vehicle_code}/{sensor_code}/data
```

## 6) Cek Node yang Aktif

```bash
ros2 node list
```

## 7) Troubleshooting Singkat

Kalau command `ros2 run` tidak menemukan executable:

1. Pastikan sudah `colcon build` tanpa error.
2. Pastikan sudah `source install/setup.bash` di terminal yang sama.
3. Cek executable yang tersedia:

```bash
ros2 pkg executables seano_cam
ros2 pkg executables seano_command
ros2 pkg executables seano_communication
ros2 pkg executables seano_failsafe
ros2 pkg executables seano_logging
ros2 pkg executables seano_mqtt_bridge
ros2 pkg executables seano_oceanography
ros2 pkg executables seano_telemetry
```

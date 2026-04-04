# seano_vision

Package collision avoidance berbasis vision (YOLO + fusion + risk + watchdog + actuation bridge MAVROS) untuk USV SEANO.

## Node Utama

- `camera_node`: ambil frame dari USB/RTSP lalu publish image ROS2.
- `detector_node`: inferensi YOLOv8, publish detections + image annotated.
- `multi_target_fusion_node`: gabungkan multi deteksi menjadi target stabil.
- `risk_evaluator_node`: hitung level risiko tabrakan.
- `actuator_safety_limiter_node`: batasi command avoidance agar aman.
- `command_mux_node`: pilih sumber command manual/auto.
- `mavros_rc_override_bridge_node`: kirim output left/right ke MAVROS RC override.
- `watchdog_failsafe_node`: fail-safe jika sensor/pipeline vision bermasalah.

## ROS2 Interface (Ringkas)

### Published

- Topic kamera/deteksi/risk/command sesuai launch file package (`launch/*.launch.py`).
- Topic final aktuasi umumnya: `/seano/left_cmd`, `/seano/right_cmd`.
- Bridge ke MAVROS: `/mavros/rc/override`.

### Subscribed

- Topic image kamera (misal `/camera/image_raw_reliable`).
- Topic command auto/manual left-right sesuai launch.
- Topic internal pipeline collision avoidance.

## MQTT

Tidak ada koneksi MQTT langsung di package ini.

## Dependency Penting

- ROS2 Humble: `rclpy`, `sensor_msgs`, `geometry_msgs`, `mavros_msgs`, `cv_bridge`, `vision_msgs`
- Python runtime vision: `numpy`, `opencv-python`, `ultralytics` (YOLOv8)
- Model default: `models/yolov8n.pt`

## Jalankan

Contoh stack auto command + bridge MAVROS:

```bash
ros2 launch seano_vision run_auto_stack.launch.py
```

Contoh camera + detector test:

```bash
ros2 launch seano_vision phase2_camera_detector_test.launch.py
```

## Mode Menjalankan (Fleksibel)

### 1) Jalan bareng semua package

Stack vision jalan saat startup global (default aktif):

```bash
ros2 launch seano_startup system.launch.py
```

Untuk mematikan vision saat startup:

```bash
ros2 launch seano_startup system.launch.py enable_vision_stack:=false
```

### 2) Jalan satu-satu (untuk debug)

```bash
ros2 launch seano_vision run_auto_stack.launch.py
ros2 launch seano_vision phase2_camera_detector_test.launch.py
```

### Cek cepat

```bash
ros2 node list | grep -E 'detector|fusion|risk|watchdog|mavros_rc_override'
```

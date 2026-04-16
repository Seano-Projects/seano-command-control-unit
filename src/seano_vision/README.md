# seano_vision

Package collision avoidance berbasis vision (YOLO + fusion + risk + watchdog + actuation bridge MAVROS) untuk USV SEANO.

## Node Utama

| Node | Keterangan |
|------|-----------|
| `camera_node` | Ambil frame dari USB/RTSP, publish image ROS 2 |
| `detector_node` | Inferensi YOLOv8, publish detections + image annotated |
| `multi_target_fusion_node` | Gabungkan multi deteksi menjadi target stabil |
| `risk_evaluator_node` | Hitung level risiko tabrakan (rendah/sedang/tinggi) |
| `actuator_safety_limiter_node` | Batasi command avoidance agar aman |
| `command_mux_node` | Pilih sumber command manual vs auto |
| `mavros_rc_override_bridge_node` | Kirim output left/right ke MAVROS RC override |
| `watchdog_failsafe_node` | Fail-safe jika sensor/pipeline vision bermasalah |
| `rtmp_streamer` | Stream video ke RTMP server |
| `waterline_horizon_node` | Deteksi waterline untuk horizon reference |
| `vision_quality_node` | Monitor kualitas frame kamera |
| `thrsteer_to_auto_left_right_node` | Konversi throttle+steering ke left/right motor |

## ROS 2 Topics

### Subscribe

| Topic | Tipe | Keterangan |
|-------|------|-----------|
| `/seano/camera/image_raw_reliable` | `sensor_msgs/Image` | Input frame kamera |
| `/camera/image_raw` | `sensor_msgs/Image` | Input alternatif (rtmp_streamer) |

### Publish

| Topic | Tipe | Keterangan |
|-------|------|-----------|
| `/camera/image_annotated` | `sensor_msgs/Image` | Frame dengan bounding box |
| `/camera/detections` | `vision_msgs/Detection2DArray` | Hasil deteksi YOLO |
| `/mavros/rc/override` | `mavros_msgs/OverrideRCIn` | Override RC ke flight controller |

## MQTT

Tidak ada koneksi MQTT langsung di package ini.

## Dependency Penting

- ROS 2 Humble: `rclpy`, `sensor_msgs`, `geometry_msgs`, `mavros_msgs`, `cv_bridge`, `vision_msgs`
- Python: `numpy`, `opencv-python`, `ultralytics` (YOLOv8)
- Model default: `models/yolov8n.pt`

## Launch Files

| Launch File | Keterangan |
|------------|-----------|
| `vision_stream.launch.py` | Camera + detector + RTMP stream |
| `phase2_camera_detector_test.launch.py` | Camera + detector saja (test) |
| `run_auto_stack.launch.py` | Stack aktuasi collision avoidance |
| `demo_full_ca.launch.py` | Full stack collision avoidance + camera |
| `phase3_watchdog_camera_only.launch.py` | Camera + watchdog |

## Cara Menjalankan

```bash
# Full system (via startup)
ros2 launch seano_startup system.launch.py

# Hanya vision stream
ros2 launch seano_vision vision_stream.launch.py

# Test camera + detector
ros2 launch seano_vision phase2_camera_detector_test.launch.py
```

## Verifikasi

```bash
ros2 node list | grep -E 'detector|fusion|risk|watchdog|mavros_rc_override'
ros2 topic echo /camera/detections
```

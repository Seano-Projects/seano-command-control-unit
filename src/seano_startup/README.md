# seano_startup

Package orchestration untuk start sistem USV secara terpusat menggunakan launch file dan parameter global.

## Fungsi Utama

- Menyediakan konfigurasi global di `config/system.yaml`
- Menjalankan MAVROS dan node-node utama package lain lewat `launch/system.launch.py`
- Memberikan parameter yang konsisten ke semua node

## Launch

- `system.launch.py` menjalankan:
  - `mavros` launch (`apm.launch`)
  - `seano_vision` full CA stack (`demo_full_ca.launch.py`)
  - `seano_vision` actuation stack (`run_auto_stack.launch.py`)
  - `seano_telemetry/telemetry_node`
  - `seano_logging/telemetry_logger_node`
  - `seano_logging/logger_node`
  - `seano_mqtt_bridge/mqtt_bridge_node`
  - `seano_mqtt_bridge/mqtt_status_node`
  - `seano_command/command_node`
  - `seano_communication/communication_node`
  - `seano_anti_theft/anti_theft_node`
  - `seano_oceanography/ctd_sensor_node`
  - `seano_failsafe/seano_battery`
  - `seano_failsafe/seano_communication_monitor`
  - `seano_failsafe/seano_failsafe`
  - `seano_cam/rtmp_streamer` (stream output vision ke RTMP)

Catatan: launch ini memakai namespace `usv`, jadi topic/node akan ter-prefix `usv`.

## Konfigurasi

- File parameter utama: `config/system.yaml`
- Isi penting: `vehicle`, `communication`, `logging`, `mqtt`, `oceanography`, `failsafe`, `anti_theft`, `collision_avoidance`, `camera`, `rtmp`

## Integrasi Kamera

- Sumber kamera utama: `seano_vision`.
- Streaming RTMP: `seano_cam/rtmp_streamer` subscribe ke `camera.topic` (default `/camera/image_annotated`).
- Dengan skema ini, tidak perlu menjalankan `seano_cam/camera_node` bersamaan dengan `seano_vision/camera_node`, sehingga bentrok device kamera bisa dihindari.

## Profil Otomatis Vision

- `system.launch.py` membaca `system.mode` dari `config/system.yaml`.
- Jika `system.mode=field_test`, launch otomatis memakai profil ringan (Jetson-friendly):
  - `vision_det_imgsz=320`
  - `vision_det_max_fps=6.0`
  - `vision_det_conf=0.30`
  - `enable_vision_actuation=false` (aman-by-default, tidak langsung override RC)
- Mode selain `field_test` memakai profil balanced:
  - `vision_det_imgsz=416`
  - `vision_det_max_fps=10.0`
  - `vision_det_conf=0.25`
  - `enable_vision_actuation=false`

Untuk mengaktifkan aktuasi collision avoidance ke RC override secara eksplisit:

```bash
ros2 launch seano_startup system.launch.py enable_vision_actuation:=true
```

Semua tetap bisa dioverride saat launch, contoh:

```bash
ros2 launch seano_startup system.launch.py vision_det_max_fps:=8.0 vision_det_imgsz:=416
```

## MQTT

Tidak ada client MQTT di package ini, tetapi package ini menyuplai parameter MQTT global untuk package lain.

## Jalankan

```bash
ros2 launch seano_startup system.launch.py
```

## Mode Menjalankan (Fleksibel)

### 1) Jalan bareng semua package (full system)

```bash
ros2 launch seano_startup system.launch.py
```

Atau pakai helper script:

```bash
cd /home/seano/Seano_ws
./start_seano.sh
```

### 2) Jalan satu-satu (per package)

Gunakan perintah `ros2 run` atau `ros2 launch` pada README masing-masing package jika ingin debug modular.

Contoh minimal:

```bash
ros2 run seano_telemetry telemetry_node --ros-args --params-file /home/seano/Seano_ws/src/seano_startup/config/system.yaml
ros2 run seano_mqtt_bridge mqtt_bridge_node --ros-args --params-file /home/seano/Seano_ws/src/seano_startup/config/system.yaml
```

### Cek cepat

```bash
ros2 node list | grep /usv/
```

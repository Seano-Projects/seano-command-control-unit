# seano_startup

Package orchestration untuk start sistem USV secara terpusat menggunakan launch file dan parameter global. Package ini juga berisi node MQTT bridge yang menghubungkan ROS 2 dengan MQTT broker.

## Fungsi Utama

- Menyediakan konfigurasi global di `config/system.yaml`
- Menjalankan MAVROS dan semua node utama lewat `launch/system.launch.py`
- Memberikan parameter yang konsisten ke semua node
- MQTT bridge (publish telemetri & failsafe) dan MQTT status (online/offline heartbeat)

## Node

| Node | Executable | Keterangan |
|------|-----------|-----------|
| `mqtt_bridge` | `mqtt_bridge_node` | Subscribe ROS → publish ke MQTT broker |
| `mqtt_status` | `mqtt_status_node` | Publish status online/offline + heartbeat ke MQTT |

## MQTT Topics

### Publish (ke broker)

| MQTT Topic | Sumber ROS | Keterangan |
|------------|-----------|-----------|
| `seano/{vehicle_id}/telemetry` | `/usv/telemetry` | JSON telemetri lengkap |
| `seano/{vehicle_id}/failsafe` | `/usv/failsafe/alert` | Alert failsafe |
| `seano/{vehicle_id}/status` | — | `"online"` / `"offline"` (retain=true) |
| `seano/{vehicle_id}/raw` | `/usv/raw/log` dan `/rosout` | Raw log stream (plain text / JSON) |

### Subscribe (dari broker)

Tidak ada subscribe MQTT di startup — perintah dari MQTT dihandle `seano_command`.

## Launch

`system.launch.py` menjalankan semua komponen berikut dalam namespace `usv`:

| Node | Package |
|------|---------|
| `mavros` | mavros |
| `seano_vision` full CA stack | seano_vision |
| `seano_vision` actuation stack | seano_vision (conditional) |
| `telemetry_node` | seano_telemetry |
| `mqtt_bridge_node` | **seano_startup** |
| `mqtt_status_node` | **seano_startup** |
| `command_node` | seano_command |
| `mission_node` | seano_mission |
| `thruster_node` | seano_command |
| `communication_node` | seano_communication |
| `anti_theft_node` | seano_anti_theft |
| `ctd_sensor_node` | seano_oceanography |
| `seano_battery` | seano_failsafe |
| `seano_communication_monitor` | seano_failsafe |
| `seano_failsafe` | seano_failsafe |
| `rtmp_streamer` | seano_vision (conditional) |
| `csv_logger_node` | seano_logger |

Catatan: launch ini memakai namespace `usv`, jadi topic/node akan ter-prefix `/usv/`.

## Konfigurasi

- File parameter utama: `config/system.yaml`
- Isi penting: `vehicle`, `communication`, `logging`, `mqtt`, `oceanography`, `failsafe`, `anti_theft`, `collision_avoidance`, `camera`, `rtmp`

## Integrasi Kamera

- Sumber kamera utama: `seano_vision`.
- Streaming RTMP: `rtmp_streamer` (dari `seano_vision`) subscribe ke `camera.topic` (default `/camera/image_raw`).
- Dengan skema ini, tidak perlu menjalankan camera node terpisah, sehingga bentrok device kamera bisa dihindari.

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

Node `mqtt_bridge_node` terhubung ke broker saat startup dan langsung forward data. Node `mqtt_status_node` dijalankan di urutan awal launch, mengirim heartbeat, dan menggunakan Last Will & Testament (LWT) MQTT sehingga broker otomatis menerima pesan `"offline"` jika koneksi putus.

`seano_mission/mission_node` menangani upload waypoint dari MQTT dan publish ACK ke `seano/{vehicle_id}/waypoint/status`.

Parameter status yang relevan di `config/system.yaml`:

- `mqtt.status_keepalive` (default: `5` detik): keepalive khusus node status. Nilai lebih kecil membuat `offline` lebih cepat terdeteksi oleh broker saat Jetson mati mendadak.
- `mqtt.heartbeat_interval` (default: `30.0` detik): interval publish `online` retain.

Catatan: untuk mati mendadak (power loss), deteksi `offline` tetap mengikuti timeout keepalive broker (bukan instant absolut), tapi dengan keepalive kecil transisinya jauh lebih cepat.

## Cara Menjalankan

```bash
# Full system
ros2 launch seano_startup system.launch.py

# Atau pakai helper script
cd ~/Seano_ws && ./start_seano.sh
```

## Auto Start Saat Boot Jetson

Supaya semua package langsung jalan saat Jetson menyala, gunakan `systemd` dan panggil helper installer dari workspace:

```bash
cd ~/Seano_ws
sudo ./scripts/install_seano_autostart.sh
```

Installer ini akan:

- membuat service `seano.service`
- mengaktifkan autostart saat boot
- tetap memakai `start_seano.sh` sebagai entrypoint utama

Perintah operasional:

```bash
sudo systemctl start seano.service
sudo systemctl restart seano.service
sudo systemctl status seano.service
journalctl -u seano.service -f
```

Untuk mengubah argumen default boot, edit:

```bash
/etc/default/seano
```

Contoh:

```bash
SEANO_START_ARGS="--no-vision"
```

Untuk debug per package, jalankan node mandiri:

```bash
ros2 run seano_telemetry telemetry_node --ros-args --params-file ~/Seano_ws/src/seano_startup/config/system.yaml
ros2 run seano_startup mqtt_bridge_node --ros-args --params-file ~/Seano_ws/src/seano_startup/config/system.yaml
```

```bash
# Cek node yang jalan
ros2 node list | grep /usv/
```

# seano_command

Node ROS2 untuk menerima command dan waypoint dari MQTT, lalu meneruskannya ke MAVROS.

## Fungsi Utama

`command_node` melakukan hal berikut:

1. Subscribe MQTT topic command:
- `seano/{vehicle_id}/command`

2. Subscribe MQTT topic waypoint:
- `seano/{vehicle_id}/waypoint`

3. Eksekusi command ke MAVROS:
- ARM / FORCE_ARM / DISARM / FORCE_DISARM lewat service `/mavros/cmd/command`
- AUTO / MANUAL / HOLD / LOITER / RTL lewat service `/mavros/set_mode`

4. Upload waypoint ke MAVROS:
- Service `/mavros/mission/push`

5. Publish status hasil eksekusi:
- ROS topic: `command_status` (std_msgs/String berisi JSON)
- MQTT topic: `seano/{vehicle_id}/status`

## Parameter ROS

Parameter dibaca dari ROS parameter server (umumnya dari `system.yaml`):

- `vehicle.id` (default: `UNKNOWN`)
- `mqtt.broker` (default: `localhost`)
- `mqtt.port` (default: `1883`)
- `mqtt.username` (default: kosong)
- `mqtt.password` (default: kosong)
- `mqtt.base_topic` (default: `seano`)
- `mqtt.qos` (default: `1`)
- `mqtt.keepalive` (default: `60`)
- `mqtt.use_tls` (default: `true`)
- `mqtt.tls_insecure` (default: `true`)
- `mission.auto_set_home_from_first_waypoint` (default: `true`)

## Format Payload MQTT

## 1) Topic command

Topic:

```text
seano/{vehicle_id}/command
```

Payload JSON:

```json
{
  "command": "ARM"
}
```

Command yang didukung:

- `ARM`
- `FORCE_ARM`
- `DISARM`
- `FORCE_DISARM`
- `AUTO`
- `MANUAL`
- `HOLD`
- `LOITER`
- `RTL`

## 2) Topic waypoint

Topic:

```text
seano/{vehicle_id}/waypoint
```

Node mendukung 3 bentuk payload:

### Bentuk A: object dengan key `waypoints`

```json
{
  "set_home_from_first_waypoint": true,
  "waypoints": [
    {"lat": -6.2001, "lon": 106.8167, "alt": 5.0},
    {"latitude": -6.2005, "longitude": 106.8172, "altitude": 5.0}
  ]
}
```

### Bentuk B: array langsung

```json
[
  {"lat": -6.2001, "lon": 106.8167, "alt": 5.0},
  {"lat": -6.2005, "lon": 106.8172, "alt": 5.0}
]
```

### Bentuk C: satu waypoint object

```json
{"lat": -6.2001, "lon": 106.8167, "alt": 5.0}
```

Field yang dikenali:

- Latitude: `latitude` atau `lat`
- Longitude: `longitude` atau `lon` atau `lng`
- Altitude: `altitude` atau `alt` (default `0.0`)
- Opsional MAVLink: `frame`, `command`, `param1`, `param2`, `param3`, `param4`, `autocontinue`

Validasi waypoint:

- Lat/lon harus ada
- Lat/lon/alt harus numerik
- Rentang lat: `-90..90`
- Rentang lon: `-180..180`
- Waypoint tidak valid akan di-skip

Catatan Home Point:

- Secara default node akan mencoba set home ke waypoint pertama sebelum upload mission.
- Per-payload bisa di-override dengan field `set_home_from_first_waypoint` (hanya untuk Bentuk A).
- Jika proses set home gagal, upload waypoint dibatalkan untuk menghindari RTL ke home yang salah.

## Tahapan dari Setup Sampai Bisa Start

Urutan cepat dari nol sampai autopilot command bisa dieksekusi:

1. Siapkan dependency ROS2 + MAVROS + broker MQTT.

2. Build package:

```bash
cd ~/Seano_ws
colcon build --packages-select seano_command
```

3. Source environment ROS2 dan workspace:

```bash
source /opt/ros/humble/setup.bash
source ~/Seano_ws/install/setup.bash
```

4. Pastikan file parameter ada dan `vehicle.id` sudah benar (default `USV-001`):

```bash
cat /home/seano/Seano_ws/src/seano_startup/config/system.yaml
```

5. Start autopilot + MAVROS (SITL atau hardware) sampai service MAVROS aktif.

6. Jalankan node command:

```bash
ros2 run seano_command command_node --ros-args --params-file /home/seano/Seano_ws/src/seano_startup/config/system.yaml
```

7. Verifikasi service MAVROS tersedia:

```bash
ros2 service list | grep mavros
```

Minimal harus ada:
- `/mavros/cmd/command`
- `/mavros/set_mode`
- `/mavros/mission/push`

8. Mulai test autopilot dari MQTT:
- Kirim command mode (misal `AUTO`) ke topic `seano/USV-001/command`
- Kirim waypoint ke topic `seano/USV-001/waypoint`
- Pantau hasil di topic `seano/USV-001/status`

9. Kalau status sukses (`Mode changed to AUTO`, `Uploaded N waypoints`), berarti flow dari command sampai autopilot sudah jalan.

Build package:

```bash
cd ~/Seano_ws
colcon build --packages-select seano_command
```

Source environment:

```bash
source ~/Seano_ws/install/setup.bash
```

Run node langsung:

```bash
ros2 run seano_command command_node --ros-args --params-file /home/seano/Seano_ws/src/seano_startup/config/system.yaml
```

Atau lewat launch sistem:

```bash
ros2 launch seano_startup system.launch.py
```

## Simulasi Fitur Autopilot (Tanpa Web)

Bagian ini untuk uji end-to-end fitur autopilot dari MQTT ke MAVROS memakai terminal.

### Prasyarat

1. Autopilot + MAVROS aktif (hardware asli atau SITL), dan service berikut tersedia:

```bash
ros2 service list | grep mavros
```

Minimal terlihat:
- `/mavros/cmd/command`
- `/mavros/set_mode`
- `/mavros/mission/push`

2. Node command sudah jalan:

```bash
ros2 run seano_command command_node --ros-args --params-file /home/seano/Seano_ws/src/seano_startup/config/system.yaml
```

3. Sesuaikan topic dengan `vehicle.id`.
Default di project ini adalah `USV-001`, sehingga topic menjadi:
- `seano/USV-001/command`
- `seano/USV-001/waypoint`
- `seano/USV-001/status`

### Langkah Uji di 3 Terminal

#### Terminal 1: monitor status dari node

```bash
mosquitto_sub -h mqtt.seano.cloud -p 8883 -u seanomqtt -P 'Seano2025*' --insecure -t 'seano/USV-001/status'
```

#### Terminal 2: kirim mode AUTO

```bash
mosquitto_pub -h mqtt.seano.cloud -p 8883 -u seanomqtt -P 'Seano2025*' --insecure -t 'seano/USV-001/command' -m '{"command":"AUTO"}'
```

#### Terminal 3: kirim waypoint

Contoh payload bentuk object dengan key `waypoints`:

```bash
mosquitto_pub -h mqtt.seano.cloud -p 8883 -u seanomqtt -P 'Seano2025*' --insecure -t 'seano/USV-001/waypoint' -m '{"waypoints":[{"lat":-6.2001,"lon":106.8167,"alt":0.0},{"lat":-6.2005,"lon":106.8172,"alt":0.0}]}'
```

### Verifikasi Berhasil

1. Di status MQTT muncul pesan sukses, misalnya:
- `Mode changed to AUTO`
- `Uploaded 2 waypoints`

2. Di log node terlihat proses subscribe, eksekusi mode, dan upload waypoint.

3. Opsional cek mission di MAVROS:

```bash
ros2 service call /mavros/mission/pull mavros_msgs/srv/WaypointPull '{}'
```

### Catatan Penting untuk USV

- Untuk kapal, navigasi waypoint utamanya berdasarkan latitude dan longitude.
- Field altitude tetap diparsing dan dikirim ke MAVROS, namun umumnya tidak dominan pada kontrol gerak USV.
- Jika ragu, isi altitude dengan `0.0` agar payload konsisten.

## Catatan Integrasi Web

Agar command/waypoint dari web masuk ke node ini:

1. Pastikan `vehicle.id` di ROS sama dengan `{vehicle_id}` yang dipakai web.
2. Publish payload JSON valid ke topic `seano/{vehicle_id}/command` atau `seano/{vehicle_id}/waypoint`.
3. Cek hasil di topic status `seano/{vehicle_id}/status`.

## File Penting

- Node utama: `seano_command/command_node.py`
- Metadata package: `package.xml`
- Entry point Python: `setup.py`

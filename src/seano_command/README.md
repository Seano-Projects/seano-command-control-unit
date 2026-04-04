# seano_command

Package ROS2 yang terdiri dari 3 node terpisah untuk menerima perintah dari MQTT dan meneruskannya ke MAVROS.

## Node di Package Ini

| Node | File | MQTT Subscribe | Fungsi |
|---|---|---|---|
| `command_node` | `command_node.py` | `seano/{vehicle_id}/command` | ARM/DISARM/ganti mode |
| `waypoint_node` | `waypoint_node.py` | `seano/{vehicle_id}/waypoint` | Upload misi waypoint |
| `thruster_node` | `thruster_node.py` | `seano/{vehicle_id}/thruster` | Kontrol PWM motor |

## Fungsi Utama

### command_node
- Subscribe MQTT topic `seano/{vehicle_id}/command`
- Eksekusi ARM / FORCE_ARM / DISARM / FORCE_DISARM lewat service `/mavros/cmd/command`
- Ganti mode AUTO / MANUAL / HOLD / LOITER / RTL lewat service `/mavros/set_mode`
- Publish status ke ROS topic `command_status` dan MQTT `seano/{vehicle_id}/command/response`

### waypoint_node
- Subscribe MQTT topic `seano/{vehicle_id}/waypoint`
- Upload waypoint ke MAVROS lewat service `/mavros/mission/push`
- Opsional set home dari waypoint pertama sebelum upload
- Publish status ke ROS topic `waypoint_status` dan MQTT `seano/{vehicle_id}/waypoint/response`

### thruster_node
- Subscribe MQTT topic `seano/{vehicle_id}/thruster`
- Publish PWM ke `/mavros/rc/override` (OverrideRCIn)
- Nilai throttle/steering: -100..100 (0 = netral, PWM 1500 µs)

## Parameter ROS

Semua parameter dibaca dari `system.yaml` via ROS parameter server.

### Parameter Umum (semua node)

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

### Parameter waypoint_node

- `mission.auto_set_home_from_first_waypoint` (default: `true`)

### Parameter thruster_node

- `thruster.pwm_neutral` (default: `1500`)
- `thruster.pwm_min` (default: `1000`)
- `thruster.pwm_max` (default: `2000`)
- `thruster.channel_throttle` (default: `2` → CH3, 0-indexed)
- `thruster.channel_steering` (default: `0` → CH1, 0-indexed)
- `thruster.allow_reverse` (default: `true`)

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

## Cara Menjalankan

### Build package

```bash
cd ~/Seano_ws
colcon build --packages-select seano_command
source ~/Seano_ws/install/setup.bash
```

### Opsi 1: Jalan semua sekaligus (rekomendasi)

Ketiga node (`command_node`, `waypoint_node`, `thruster_node`) sudah terdaftar di launch sistem dan otomatis ikut jalan:

```bash
ros2 launch seano_startup system.launch.py
```

### Opsi 2: Jalan hanya seano_command (tanpa package lain)

Gunakan launch file bawaan package ini:

```bash
ros2 launch seano_command command.launch.py
```

Ini menjalankan ketiga node sekaligus (`command_node`, `waypoint_node`, `thruster_node`) tanpa perlu membuka 3 terminal terpisah.

Bisa juga tentukan file parameter secara eksplisit:

```bash
ros2 launch seano_command command.launch.py params_file:=~/Seano_ws/src/seano_startup/config/system.yaml
```

### Opsi 3: Jalan manual satu per satu (untuk debug per node)

Buka 3 terminal terpisah, jalankan masing-masing:

**Terminal 1 — command_node (ARM/DISARM/mode):**

```bash
ros2 run seano_command command_node --ros-args --params-file ~/Seano_ws/src/seano_startup/config/system.yaml
```

**Terminal 2 — waypoint_node (upload misi):**

```bash
ros2 run seano_command waypoint_node --ros-args --params-file ~/Seano_ws/src/seano_startup/config/system.yaml
```

**Terminal 3 — thruster_node (kontrol PWM motor):**

```bash
ros2 run seano_command thruster_node --ros-args --params-file ~/Seano_ws/src/seano_startup/config/system.yaml
```

### Cek node aktif

```bash
ros2 node list | grep -E 'command|waypoint|thruster'
```

Harus muncul:
- `/usv/command`
- `/usv/waypoint`
- `/usv/thruster`

### Verifikasi service MAVROS tersedia

```bash
ros2 service list | grep mavros
```

Minimal harus ada:
- `/mavros/cmd/command`
- `/mavros/set_mode`
- `/mavros/mission/push`



## Simulasi / Test dari MQTT (Tanpa Web)

Ketiga node harus sudah jalan (via launch atau manual). Default `vehicle.id` adalah `USV-001`.

### Uji command_node

**Monitor response:**
```bash
mosquitto_sub -h mqtt.seano.cloud -p 8883 -u seanomqtt -P 'Seano2025*' --insecure -t 'seano/USV-001/command/response'
```

**Kirim ARM:**
```bash
mosquitto_pub -h mqtt.seano.cloud -p 8883 -u seanomqtt -P 'Seano2025*' --insecure -t 'seano/USV-001/command' -m '{"command":"ARM"}'
```

**Kirim mode AUTO:**
```bash
mosquitto_pub -h mqtt.seano.cloud -p 8883 -u seanomqtt -P 'Seano2025*' --insecure -t 'seano/USV-001/command' -m '{"command":"AUTO"}'
```

### Uji waypoint_node

**Monitor response:**
```bash
mosquitto_sub -h mqtt.seano.cloud -p 8883 -u seanomqtt -P 'Seano2025*' --insecure -t 'seano/USV-001/waypoint/response'
```

**Kirim waypoint:**
```bash
mosquitto_pub -h mqtt.seano.cloud -p 8883 -u seanomqtt -P 'Seano2025*' --insecure -t 'seano/USV-001/waypoint' -m '{"waypoints":[{"lat":-6.2001,"lon":106.8167,"alt":0.0},{"lat":-6.2005,"lon":106.8172,"alt":0.0}]}'
```

### Uji thruster_node

**Maju 60%, belok kiri 20%:**
```bash
mosquitto_pub -h mqtt.seano.cloud -p 8883 -u seanomqtt -P 'Seano2025*' --insecure -t 'seano/USV-001/thruster' -m '{"throttle":60,"steering":-20}'
```

**Berhenti (netral):**
```bash
mosquitto_pub -h mqtt.seano.cloud -p 8883 -u seanomqtt -P 'Seano2025*' --insecure -t 'seano/USV-001/thruster' -m '{"throttle":0,"steering":0}'
```

**Lepas override (kembalikan ke RC fisik):**
```bash
mosquitto_pub -h mqtt.seano.cloud -p 8883 -u seanomqtt -P 'Seano2025*' --insecure -t 'seano/USV-001/thruster' -m '{"release":true}'
```

### Cek mission di MAVROS (setelah upload waypoint)

```bash
ros2 service call /mavros/mission/pull mavros_msgs/srv/WaypointPull '{}'
```

### Catatan Penting untuk USV

- Navigasi waypoint utamanya berdasarkan latitude dan longitude, isi altitude `0.0`.
- Thruster nilai `0` = netral (PWM 1500 µs), bukan berhenti mendadak.

## Catatan Integrasi Web

Agar command/waypoint dari web masuk ke node ini:

1. Pastikan `vehicle.id` di ROS sama dengan `{vehicle_id}` yang dipakai web.
2. Publish payload JSON valid ke topic `seano/{vehicle_id}/command` atau `seano/{vehicle_id}/waypoint`.
3. Cek hasil di topic status `seano/{vehicle_id}/status`.

## File Penting

| File | Keterangan |
|---|---|
| `seano_command/command_node.py` | Node ARM/DISARM/mode |
| `seano_command/waypoint_node.py` | Node upload waypoint |
| `seano_command/thruster_node.py` | Node kontrol PWM motor |
| `package.xml` | Metadata package |
| `setup.py` | Entry point ketiga node |

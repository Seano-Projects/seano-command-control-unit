# seano_command

Package ROS2 yang berisi 3 node untuk menerima perintah dari web dashboard via MQTT dan meneruskannya ke flight controller (ArduPilot/ArduRover) lewat MAVROS.

---

## Daftar Isi

- [Ringkasan Node](#ringkasan-node)
- [Alur Kerja](#alur-kerja)
- [command_node](#command_node)
- [waypoint_node](#waypoint_node)
- [thruster_node](#thruster_node)
- [Parameter Konfigurasi](#parameter-konfigurasi)
- [Cara Menjalankan](#cara-menjalankan)
- [Monitoring & Debug](#monitoring--debug)
- [Test dari Terminal (tanpa web)](#test-dari-terminal-tanpa-web)
- [Struktur Package](#struktur-package)
- [Troubleshooting](#troubleshooting)

---

## Ringkasan Node

| Node | File | MQTT Subscribe | MAVROS Service | Fungsi |
|---|---|---|---|---|
| `command_node` | `command_node.py` | `seano/{id}/command` | `/mavros/cmd/command`, `/mavros/set_mode` | ARM/DISARM/ganti mode flight |
| `waypoint_node` | `waypoint_node.py` | `seano/{id}/waypoint` | `/mavros/mission/push`, `/mavros/cmd/command` | Upload misi waypoint ke FC |
| `thruster_node` | `thruster_node.py` | `seano/{id}/thruster` | — (publish ke topic) | Kontrol throttle & steering PWM |

---

## Alur Kerja

```
Web Dashboard
     │
     │  MQTT Publish
     ▼
MQTT Broker
     │
     ├──► seano/{id}/command  ──► command_node  ──► /mavros/set_mode
     │                                          ──► /mavros/cmd/command (ARM/DISARM)
     │                                          ──► command_status (ROS)
     │                                          ──► seano/{id}/command/response (MQTT)
     │
     ├──► seano/{id}/waypoint ──► waypoint_node ──► /mavros/cmd/command (set_home)
     │                                          ──► /mavros/mission/push
     │                                          ──► waypoint_status (ROS)
     │                                          ──► seano/{id}/waypoint/response (MQTT)
     │
     └──► seano/{id}/thruster ──► thruster_node ──► /mavros/rc/override (10 Hz)
```

---

## command_node

### Fungsi
Menerima perintah ARM/DISARM dan pergantian mode flight dari web, lalu meneruskan ke flight controller lewat MAVROS service.

### Input (MQTT Subscribe)

| Topic | Format |
|---|---|
| `seano/{vehicle_id}/command` | JSON `{"command": "..."}` |

**Command yang didukung:**

| Command | Aksi |
|---|---|
| `ARM` | Arm motor (normal) |
| `FORCE_ARM` | Force arm (abaikan pre-arm check) |
| `DISARM` | Disarm motor (normal) |
| `FORCE_DISARM` | Force disarm (darurat) |
| `AUTO` | Ganti mode ke AUTO (jalankan misi waypoint) |
| `MANUAL` | Ganti mode ke MANUAL |
| `HOLD` | Ganti mode ke HOLD (berhenti di tempat) |
| `LOITER` | Ganti mode ke LOITER |
| `RTL` | Ganti mode ke RTL (Return to Launch) |

**Contoh payload:**
```json
{"command": "ARM"}
{"command": "AUTO"}
{"command": "RTL"}
```

### Output

| Tujuan | Topic | Format | Kapan |
|---|---|---|---|
| ROS | `command_status` | String JSON | Setelah MAVROS merespons |
| MQTT | `seano/{vehicle_id}/command/response` | JSON | Setelah MAVROS merespons |

**Contoh response:**
```json
{"status": "success", "message": "ARM successful", "vehicle_id": "USV-001"}
{"status": "error", "message": "MAVROS service unavailable", "vehicle_id": "USV-001"}
```

### MAVROS Services yang digunakan
- `/mavros/cmd/command` (`mavros_msgs/CommandLong`) — ARM/DISARM
- `/mavros/set_mode` (`mavros_msgs/SetMode`) — ganti mode

---

## waypoint_node

### Fungsi
Menerima daftar waypoint dari web, opsional set home dari waypoint pertama, lalu upload mission ke flight controller lewat MAVROS.

### Input (MQTT Subscribe)

| Topic | Format |
|---|---|
| `seano/{vehicle_id}/waypoint` | JSON — 3 bentuk diterima |

**Bentuk A — object dengan key `waypoints` (paling lengkap):**
```json
{
  "set_home_from_first_waypoint": true,
  "waypoints": [
    {"lat": -6.2001, "lon": 106.8167, "alt": 0.0},
    {"lat": -6.2005, "lon": 106.8172, "alt": 0.0}
  ]
}
```

**Bentuk B — array langsung:**
```json
[
  {"lat": -6.2001, "lon": 106.8167, "alt": 0.0},
  {"lat": -6.2005, "lon": 106.8172, "alt": 0.0}
]
```

**Bentuk C — satu waypoint:**
```json
{"lat": -6.2001, "lon": 106.8167, "alt": 0.0}
```

**Field waypoint yang dikenali:**

| Field | Alias | Keterangan |
|---|---|---|
| `lat` | `latitude` | Latitude (wajib) |
| `lon` | `longitude`, `lng` | Longitude (wajib) |
| `alt` | `altitude` | Altitude meter (default: `0.0`) |
| `frame` | — | MAVLink frame (default: `3` = FRAME_GLOBAL_REL_ALT) |
| `command` | — | MAVLink command (default: `16` = NAV_WAYPOINT) |
| `param1` | — | Acceptance radius meter (default: `0.0`) |
| `autocontinue` | — | Lanjut ke waypoint berikutnya (default: `true`) |

**Validasi:**
- Lat harus `-90..90`, lon harus `-180..180`
- Waypoint dengan koordinat tidak valid di-skip, sisanya tetap diupload

### Logika set home
1. Default: waypoint pertama dijadikan home point sebelum upload (`auto_set_home_from_first_waypoint: true`)
2. Bisa di-override per-payload dengan field `"set_home_from_first_waypoint": false`
3. Jika set home gagal → upload dibatalkan (mencegah RTL ke home yang salah)

### Output

| Tujuan | Topic | Format | Kapan |
|---|---|---|---|
| ROS | `waypoint_status` | String JSON | Setelah upload selesai |
| MQTT | `seano/{vehicle_id}/waypoint/response` | JSON | Setelah upload selesai |

**Contoh response:**
```json
{"status": "success", "message": "Upload 3 waypoint berhasil", "vehicle_id": "USV-001"}
{"status": "error", "message": "Service waypoint tidak tersedia", "vehicle_id": "USV-001"}
```

### MAVROS Services yang digunakan
- `/mavros/cmd/command` (`mavros_msgs/CommandLong`) — set home (MAV_CMD_DO_SET_HOME)
- `/mavros/mission/push` (`mavros_msgs/WaypointPush`) — upload waypoint list

---

## thruster_node

### Fungsi
Menerima perintah throttle & steering dari web, konversi ke nilai PWM µs, lalu publish ke `/mavros/rc/override` secara periodik 10 Hz agar flight controller tidak timeout override.

### Input (MQTT Subscribe)

| Topic | Format |
|---|---|
| `seano/{vehicle_id}/thruster` | JSON `{"throttle": N, "steering": N}` |

**Payload gerak:**
```json
{"throttle": 60, "steering": -20}
```

| Field | Rentang | Keterangan |
|---|---|---|
| `throttle` | `-100..100` | Positif = maju, negatif = mundur, `0` = netral |
| `steering` | `-100..100` | Negatif = belok kiri, positif = belok kanan, `0` = lurus |

**Payload lepas override (kembalikan ke RC fisik):**
```json
{"release": true}
```

**Rumus konversi ke PWM:**

$$\text{PWM} = \begin{cases} 1500 + \frac{\text{value}}{100} \times (2000 - 1500) & \text{jika value} \geq 0 \\ 1500 + \frac{\text{value}}{100} \times (1500 - 1000) & \text{jika value} < 0 \end{cases}$$

### Output (ROS Publish)

| Topic | Tipe | Kapan | Frekuensi |
|---|---|---|---|
| `/mavros/rc/override` | `mavros_msgs/OverrideRCIn` | Saat override aktif | 10 Hz (timer resend) |

**Mapping channel (default ArduRover):**
- CH1 (index 0) = Steering
- CH3 (index 2) = Throttle

Channel lain di-set `CHAN_NOCHANGE` (tidak mengintervensi).

---

## Parameter Konfigurasi

Konfigurasi di `~/Seano_ws/src/seano_startup/config/system.yaml`:

```yaml
# Parameter MQTT (shared semua node)
mqtt:
  broker: <broker_host>
  port: 8883
  username: <username>
  password: <password>
  base_topic: seano
  qos: 1
  keepalive: 60
  use_tls: true
  tls_insecure: true

# Parameter thruster_node
thruster:
  pwm_neutral: 1500
  pwm_min: 1000
  pwm_max: 2000
  channel_throttle: 2    # CH3 (0-indexed)
  channel_steering: 0    # CH1 (0-indexed)
  allow_reverse: true

# Parameter waypoint_node
mission:
  auto_set_home_from_first_waypoint: true
```

---

## Cara Menjalankan

### Prasyarat
```bash
cd ~/Seano_ws
colcon build --base-paths src --packages-select seano_command
source ~/Seano_ws/install/setup.bash
```

MAVROS harus berjalan dan terhubung ke flight controller.

---

### 1. Jalankan semua sekaligus via launch (rekomendasi)

```bash
source ~/Seano_ws/install/setup.bash
ros2 launch seano_command command.launch.py
```

Menjalankan `command_node`, `waypoint_node`, dan `thruster_node` sekaligus.

Atau dengan parameter file eksplisit:
```bash
ros2 launch seano_command command.launch.py \
  params_file:=~/Seano_ws/src/seano_startup/config/system.yaml
```

---

### 2. Jalankan bersama sistem penuh (via seano_startup)

```bash
bash ~/Seano_ws/start_seano.sh
```

Ketiga node otomatis ikut berjalan.

---

### 3. Jalankan node per node (untuk debug)

**Terminal 1 — command_node:**
```bash
source ~/Seano_ws/install/setup.bash
ros2 run seano_command command_node \
  --ros-args --params-file ~/Seano_ws/src/seano_startup/config/system.yaml
```

**Terminal 2 — waypoint_node:**
```bash
source ~/Seano_ws/install/setup.bash
ros2 run seano_command waypoint_node \
  --ros-args --params-file ~/Seano_ws/src/seano_startup/config/system.yaml
```

**Terminal 3 — thruster_node:**
```bash
source ~/Seano_ws/install/setup.bash
ros2 run seano_command thruster_node \
  --ros-args --params-file ~/Seano_ws/src/seano_startup/config/system.yaml
```

---

## Monitoring & Debug

### Pantau status command
```bash
ros2 topic echo command_status
```

### Pantau status waypoint
```bash
ros2 topic echo waypoint_status
```

### Pantau RC override (output thruster)
```bash
ros2 topic echo /mavros/rc/override
```

### Cek node aktif
```bash
ros2 node list | grep -E 'command|waypoint|thruster'
```

### Cek MAVROS services tersedia
```bash
ros2 service list | grep mavros
# Harus ada: /mavros/cmd/command, /mavros/set_mode, /mavros/mission/push
```

---

## Test dari Terminal (tanpa web)

Ganti `<broker>`, `<port>`, `<user>`, `<pass>` sesuai config MQTT di `system.yaml`.

### Test command_node

**Monitor response:**
```bash
mosquitto_sub -h <broker> -p <port> -u <user> -P <pass> --insecure \
  -t 'seano/USV-001/command/response'
```

**Kirim ARM:**
```bash
mosquitto_pub -h <broker> -p <port> -u <user> -P <pass> --insecure \
  -t 'seano/USV-001/command' -m '{"command":"ARM"}'
```

**Kirim mode AUTO:**
```bash
mosquitto_pub -h <broker> -p <port> -u <user> -P <pass> --insecure \
  -t 'seano/USV-001/command' -m '{"command":"AUTO"}'
```

### Test waypoint_node

**Monitor response:**
```bash
mosquitto_sub -h <broker> -p <port> -u <user> -P <pass> --insecure \
  -t 'seano/USV-001/waypoint/response'
```

**Kirim 2 waypoint:**
```bash
mosquitto_pub -h <broker> -p <port> -u <user> -P <pass> --insecure \
  -t 'seano/USV-001/waypoint' \
  -m '{"waypoints":[{"lat":-6.2001,"lon":106.8167,"alt":0.0},{"lat":-6.2005,"lon":106.8172,"alt":0.0}]}'
```

**Cek misi berhasil diupload ke MAVROS:**
```bash
ros2 service call /mavros/mission/pull mavros_msgs/srv/WaypointPull '{}'
```

### Test thruster_node

**Maju 60%, belok kiri 20%:**
```bash
mosquitto_pub -h <broker> -p <port> -u <user> -P <pass> --insecure \
  -t 'seano/USV-001/thruster' -m '{"throttle":60,"steering":-20}'
```

**Berhenti (netral):**
```bash
mosquitto_pub -h <broker> -p <port> -u <user> -P <pass> --insecure \
  -t 'seano/USV-001/thruster' -m '{"throttle":0,"steering":0}'
```

**Lepas override (kembalikan ke RC fisik):**
```bash
mosquitto_pub -h <broker> -p <port> -u <user> -P <pass> --insecure \
  -t 'seano/USV-001/thruster' -m '{"release":true}'
```

---

## Struktur Package

```
seano_command/
├── launch/
│   └── command.launch.py       # Launch ketiga node sekaligus
├── seano_command/
│   ├── __init__.py
│   ├── command_node.py         # ARM/DISARM/mode flight
│   ├── waypoint_node.py        # Upload misi waypoint
│   └── thruster_node.py        # Kontrol PWM throttle & steering
├── package.xml
├── setup.py
└── README.md
```

---

## Troubleshooting

| Masalah | Penyebab | Solusi |
|---|---|---|
| Node tidak connect ke MQTT | Broker tidak bisa diakses atau TLS config salah | Cek `mqtt.broker`, `mqtt.port`, `mqtt.use_tls` di system.yaml |
| ARM gagal — "MAVROS service unavailable" | MAVROS belum jalan atau belum konek ke FC | Jalankan MAVROS, tunggu `/mavros/cmd/command` tersedia |
| Waypoint upload gagal — "Set home gagal" | FC tidak merespons command set home | Cek koneksi FC, coba set `auto_set_home_from_first_waypoint: false` |
| Thruster tidak bergerak | `/mavros/rc/override` tidak diterima | Pastikan mode MANUAL di FC, cek channel mapping di config |
| Node crash saat start | Vehicle ID `UNKNOWN` atau MQTT gagal konek | Set `vehicle.id` di system.yaml dan pastikan broker reachable |


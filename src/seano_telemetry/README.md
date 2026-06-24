# seano_telemetry

Package ROS2 yang bertugas sebagai **pusat pengumpul data sensor** dari flight controller (ArduPilot/ArduRover via MAVROS). Node ini mengkonsolidasikan data state, GPS, IMU, baterai, kecepatan, dan suhu menjadi satu paket JSON yang dipublish setiap 1 detik dan dikonsumsi oleh package lain (MQTT bridge, logger, CSV logger).

---

## Daftar Isi

- [Ringkasan](#ringkasan)
- [Alur Kerja](#alur-kerja)
- [Input — MAVROS Subscriptions](#input--mavros-subscriptions)
- [Output — Telemetry JSON](#output--telemetry-json)
- [Logika Internal](#logika-internal)
- [Integrasi dengan Package Lain](#integrasi-dengan-package-lain)
- [Parameter Konfigurasi](#parameter-konfigurasi)
- [Cara Menjalankan](#cara-menjalankan)
- [Monitoring & Debug](#monitoring--debug)
- [Troubleshooting](#troubleshooting)

---

## Ringkasan

| Atribut                      | Detail                                                       |
| ---------------------------- | ------------------------------------------------------------ |
| Node                         | `telemetry_node`                                             |
| File                         | `seano_telemetry/telemetry_node.py`                          |
| Entry Point                  | `telemetry_node = seano_telemetry.telemetry_node:main`       |
| Publish ke                   | `telemetry` (String JSON, 1 Hz)                              |
| Namespace saat system launch | `/usv/telemetry`                                             |
| Subscribe dari               | 7 topic MAVROS                                               |
| MQTT langsung                | **Tidak** — diteruskan oleh `seano_startup/mqtt_bridge_node` |

---

## Alur Kerja

```
Flight Controller (ArduPilot/ArduRover)
          │
          │  Serial/UDP MAVLink
          ▼
       MAVROS
          │
   ┌──────┴───────────────────────────────┐
   │                                       │
   │  /mavros/state                        │
   │  /mavros/global_position/global       │
   │  /mavros/imu/data                     │
   │  /mavros/battery                      │  ◄── MAVROS topics
   │  /mavros/vfr_hud                      │
   │  /sys/.../thermal_zone*/temp          │  ◄── Jetson sysfs
   │                                       │
   └──────────────┬────────────────────────┘
                  │
                  ▼
          [telemetry_node]
          (konsolidasikan data,
           konversi quaternion→Euler,
           format JSON)
                  │
                  │  publish 1 Hz
                  ▼
           topic: telemetry
                  │
     ┌────────────┼──────────────┐
     │            │              │
     ▼            ▼              ▼
seano_startup  seano_logger   (custom)
(mqtt_bridge   (→ CSV log)    subscriber
 → MQTT)
```

---

## Input — MAVROS Subscriptions

| Topic                                   | Tipe                       | QoS                          | Data yang Diambil                                                          |
| --------------------------------------- | -------------------------- | ---------------------------- | -------------------------------------------------------------------------- |
| `/mavros/state`                         | `mavros_msgs/State`        | Reliable, depth 10           | `armed`, `mode`, `connected`                                               |
| `/mavros/global_position/global`        | `sensor_msgs/NavSatFix`    | Best Effort, depth 10        | `latitude`, `longitude`, `altitude`, `status.status` (GPS fix)             |
| `/mavros/global_position/rel_alt`       | `std_msgs/Float64`         | Best Effort, depth 10        | `altitude` relatif terhadap home (diprioritaskan)                          |
| `/mavros/imu/data`                      | `sensor_msgs/Imu`          | Best Effort, depth 10        | `orientation` (quaternion) → konversi ke `roll`, `pitch`, `yaw`, `heading` |
| `/mavros/battery`                       | `sensor_msgs/BatteryState` | Reliable, depth 10           | `voltage`, `current`, `percentage`                                         |
| `/mavros/vfr_hud`                       | `mavros_msgs/VfrHud`       | Reliable, depth 10           | `groundspeed` → `speed`, `altitude` (baro)                                 |
| `/sys/class/thermal/thermal_zone*/temp` | File sistem Linux          | Dibaca langsung tiap publish | Ambil nilai tertinggi dari semua thermal zone → `temperature_system`       |

**Catatan QoS:** Topic GPS dan IMU menggunakan `BEST_EFFORT` karena MAVROS memang mempublish dengan QoS tersebut. Mismatch QoS akan membuat subscription tidak menerima data samapai sekali pun.

---

## Output — Telemetry JSON

### ROS Topic

| Topic            | Tipe              | Frekuensi                                               |
| ---------------- | ----------------- | ------------------------------------------------------- |
| `telemetry`      | `std_msgs/String` | 1 Hz (timer 1.0 detik)                                  |
| `/usv/telemetry` | `std_msgs/String` | 1 Hz (saat jalan via system.launch.py dengan namespace) |

### Format Payload JSON

```json
{
  "vehicle_code": "USV-001",
  "battery_voltage": 25.2,
  "battery_current": 3.1,
  "battery_percentage": 75.0,
  "latitude": -6.200123,
  "longitude": 106.8167,
  "altitude": 12.5,
  "heading": 270.0,
  "armed": false,
  "gps_ok": true,
  "system_status": "OK",
  "mode": "HOLD",
  "speed": 1.3,
  "roll": 0.5,
  "pitch": -0.2,
  "yaw": 270.0,
  "temperature_system": "42.3",
  "network_iface": "enP8p1s0",
  "download_mbps": 12.34,
  "upload_mbps": 3.21
}
```

### Penjelasan Field

| Field                | Satuan        | Sumber MAVROS                                                                                            | Keterangan                                                                         |
| -------------------- | ------------- | -------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------- |
| `vehicle_code`       | —             | param `vehicle.id`                                                                                       | ID kendaraan, dari config                                                          |
| `battery_voltage`    | V             | `/mavros/battery` → `voltage`                                                                            | Dibulatkan 1 desimal                                                               |
| `battery_current`    | A             | `/mavros/battery` → `current`                                                                            | Dibulatkan 1 desimal                                                               |
| `battery_percentage` | %             | `/mavros/battery` → `percentage × 100`                                                                   | Dibulatkan 1 desimal                                                               |
| `latitude`           | derajat       | `/mavros/global_position/global`                                                                         | Dibulatkan 6 desimal                                                               |
| `longitude`          | derajat       | `/mavros/global_position/global`                                                                         | Dibulatkan 6 desimal                                                               |
| `altitude`           | m             | `/mavros/vfr_hud` (utama), fallback `/mavros/global_position/rel_alt` → `/mavros/global_position/global` | Altitude baro (selaras MP); fallback rel_alt/MSL jika VFR belum tersedia           |
| `heading`            | derajat 0–360 | `/mavros/imu/data` (yaw)                                                                                 | Yaw ternormalisasi ke 0–360°                                                       |
| `armed`              | bool          | `/mavros/state` → `armed`                                                                                | `true` jika motor di-arm                                                           |
| `gps_ok`             | bool          | `/mavros/global_position/global` → `status.status >= telemetry.gps_fix_min`                              | `true` jika status GPS >= `telemetry.gps_fix_min`                                  |
| `system_status`      | string        | `/mavros/state` → `connected`                                                                            | `"OK"` atau `"DISCONNECTED"`                                                       |
| `mode`               | string        | `/mavros/state` → `mode`                                                                                 | Mode ArduPilot: MANUAL, AUTO, HOLD, dll                                            |
| `speed`              | m/s           | `/mavros/vfr_hud` → `groundspeed`                                                                        | Kecepatan terhadap tanah                                                           |
| `roll`               | derajat       | `/mavros/imu/data` (quaternion)                                                                          | Dibulatkan 1 desimal                                                               |
| `pitch`              | derajat       | `/mavros/imu/data` (quaternion)                                                                          | Dibulatkan 1 desimal                                                               |
| `yaw`                | derajat       | `/mavros/imu/data` (quaternion)                                                                          | Dibulatkan 1 desimal                                                               |
| `temperature_system` | °C            | `/sys/class/thermal/thermal_zone*/temp`                                                                  | Suhu tertinggi dari thermal zone Jetson (CPU), dibaca tiap publish, sebagai string |
| `network_iface`      | —             | `/sys/class/net/<iface>/statistics/*`                                                                    | Nama interface yang dipantau (dari param `communication.ethernet_interface`)       |
| `download_mbps`      | Mbps          | `/sys/class/net/<iface>/statistics/rx_bytes`                                                             | Throughput download rata-rata per publish                                          |
| `upload_mbps`        | Mbps          | `/sys/class/net/<iface>/statistics/tx_bytes`                                                             | Throughput upload rata-rata per publish                                            |

---

## Logika Internal

### Konversi Quaternion → Euler

Node tidak pakai library `tf_transformations` — konversi dilakukan manual agar tidak ada dependency tambahan:

$$\text{roll} = \arctan2\left(2(w \cdot x + y \cdot z),\ 1 - 2(x^2 + y^2)\right)$$

$$\text{pitch} = \arcsin\left(2(w \cdot y - z \cdot x)\right)$$

$$\text{yaw} = \arctan2\left(2(w \cdot z + x \cdot y),\ 1 - 2(y^2 + z^2)\right)$$

Yaw kemudian dinormalisasi ke 0–360°:

```python
if yaw < 0:
    yaw += 360.0
heading = yaw
```

### GPS Fix Detection

```python
# NavSatFix status: -1=no fix, 0=fix, 1=SBAS fix, 2=GBAS fix
gps_ok = (msg.status.status >= gps_fix_min)
```

Default `telemetry.gps_fix_min = 1` (SBAS/GBAS). Set ke `0` jika ingin `gps_ok = True` saat fix biasa.

### Battery Percentage

MAVROS mempublish `percentage` dalam rentang `0.0–1.0`. Node mengkonversi ke persen:

```python
battery_percentage = round(msg.percentage * 100, 1) if msg.percentage >= 0 else 0.0
```

Jika MAVROS tidak punya data baterai (`percentage = -1`), di-set ke `0.0`.

---

## Integrasi dengan Package Lain

| Package            | Cara Menggunakan Telemetry                               | Topic                                |
| ------------------ | -------------------------------------------------------- | ------------------------------------ |
| `seano_startup`    | Subscribe `telemetry`, forward ke MQTT broker            | `telemetry` → `seano/{id}/telemetry` |
| `seano_logger`     | Subscribe `telemetry`, log ke CSV subfolder `telemetry/` | `telemetry`                          |
| `seano_anti_theft` | Tidak subscribe — punya GPS/IMU subscribe sendiri        | —                                    |

---

## Parameter Konfigurasi

Semua parameter dibaca dari `system.yaml` via `--params-file`:

| Parameter                          | Default     | Keterangan                                                                                        |
| ---------------------------------- | ----------- | ------------------------------------------------------------------------------------------------- |
| `vehicle.id`                       | `"USV-001"` | ID kendaraan, masuk ke field `vehicle_code` di JSON                                               |
| `system.mode`                      | `"unknown"` | Mode operasi sistem (field, test, dev) — tersimpan di node tapi tidak masuk payload JSON saat ini |
| `communication.ethernet_interface` | `""`        | Interface LAN untuk hitung Mbps (contoh: `enP8p1s0`)                                              |
| `telemetry.gps_fix_min`            | `1`         | Minimum NavSatFix status untuk dianggap `gps_ok` (0=fix, 1=SBAS, 2=GBAS)                          |

Konfigurasi di `src/seano_startup/config/system.yaml`:

```yaml
/**:
  ros__parameters:
    vehicle:
      id: USV-001
    system:
      mode: field_test
```

---

## Cara Menjalankan

### Prasyarat

1. **MAVROS harus berjalan** dan terhubung ke flight controller:

   ```bash
   # Cek MAVROS aktif
   ros2 node list | grep mavros
   # Cek topic GPS sudah ada
   ros2 topic list | grep mavros/global_position
   ```

2. Build dan source package:
   ```bash
   cd ~/Seano_ws
   colcon build --base-paths src --packages-select seano_telemetry
   source ~/Seano_ws/install/setup.bash
   ```

---

### 1. Jalankan bersama sistem penuh (rekomendasi produksi)

```bash
source ~/Seano_ws/install/setup.bash
ros2 launch seano_startup system.launch.py
```

Node berjalan di namespace `/usv`, topic menjadi `/usv/telemetry`.

Atau lewat script startup:

```bash
bash ~/Seano_ws/start_seano.sh
```

---

### 2. Jalankan node saja (debug / standalone)

```bash
source ~/Seano_ws/install/setup.bash
ros2 run seano_telemetry telemetry_node \
  --ros-args --params-file ~/Seano_ws/src/seano_startup/config/system.yaml
```

Topic: `telemetry` (tanpa namespace `/usv/`).

---

### 3. Jalankan dengan parameter override

```bash
source ~/Seano_ws/install/setup.bash
ros2 run seano_telemetry telemetry_node \
  --ros-args \
  --params-file ~/Seano_ws/src/seano_startup/config/system.yaml \
  -p vehicle.id:=USV-002 \
  -p system.mode:=dev
```

---

### 4. Jalankan dengan namespace manual (simulasi system.launch.py)

```bash
source ~/Seano_ws/install/setup.bash
ros2 run seano_telemetry telemetry_node \
  --ros-args \
  --params-file ~/Seano_ws/src/seano_startup/config/system.yaml \
  --remap __ns:=/usv
```

Topic menjadi `/usv/telemetry`.

---

## Monitoring & Debug

### Pantau output telemetry (raw)

```bash
# Tanpa namespace (saat run standalone)
ros2 topic echo telemetry

# Dengan namespace (saat via system.launch.py)
ros2 topic echo /usv/telemetry
```

### Cek frekuensi publish

```bash
ros2 topic hz telemetry
# Expected: average rate: 1.000 Hz
```

### Lihat payload JSON yang terbaca

```bash
ros2 topic echo telemetry --no-arr | python3 -c "
import sys, json
for line in sys.stdin:
    if 'data:' in line:
        data = line.split('data: ', 1)[1].strip()
        print(json.dumps(json.loads(data), indent=2))
"
```

### Cek node aktif dan topic

```bash
ros2 node list | grep telemetry
ros2 node info /telemetry_node
ros2 topic list | grep telemetry
```

### Inspeksi data per sumber MAVROS

```bash
# GPS
ros2 topic echo /mavros/global_position/global

# IMU
ros2 topic echo /mavros/imu/data

# Battery
ros2 topic echo /mavros/battery

# State
ros2 topic echo /mavros/state

# Speed (VFR HUD)
ros2 topic echo /mavros/vfr_hud

# RSSI
ros2 topic echo /mavros/radio_status

# Suhu Jetson langsung dari thermal zone
cat /sys/class/thermal/thermal_zone*/temp
# Hasil dalam milicelsius, bagi 1000 untuk dapat °C
```

---

## Troubleshooting

| Masalah                               | Penyebab                                               | Solusi                                                                                                 |
| ------------------------------------- | ------------------------------------------------------ | ------------------------------------------------------------------------------------------------------ |
| `latitude: 0.0, longitude: 0.0` terus | GPS subscription tidak menerima data                   | Cek QoS — `/mavros/global_position/global` harus pakai `BEST_EFFORT`; pastikan MAVROS terkoneksi ke FC |
| `roll`, `pitch`, `yaw` selalu `0.0`   | IMU subscription tidak menerima data                   | Cek QoS `/mavros/imu/data` harus `BEST_EFFORT`; pastikan FC mengirim IMU data                          |
| `battery_voltage: 0.0` terus          | `/mavros/battery` tidak ada data dari FC               | Cek apakah FC dikonfigurasi mengirim BATTERY_STATUS MAVLink message                                    |
| `system_status: "DISCONNECTED"`       | MAVROS tidak terhubung ke FC                           | Pastikan MAVROS berjalan dan FC power on; cek `/mavros/state` → `connected: true`                      |
| `temperature_system: "0.0"` terus     | Tidak ada path `/sys/class/thermal/thermal_zone*/temp` | Pastikan berjalan di Jetson; path thermal zone harus ada                                               |
| Node tidak publish sama sekali        | Timer tidak jalan atau node crash saat init            | Cek log: `ros2 run seano_telemetry telemetry_node --ros-args ...` dan lihat output error               |
| Topic `/usv/telemetry` tidak ada      | Node dirun tanpa namespace                             | Gunakan `system.launch.py` atau tambah `--remap __ns:=/usv`                                            |
| `gps_ok: false` padahal GPS ada fix   | FC mengirim `status.status = -1` (SERVICE_UNKNOWN)     | Cek kondisi FC dan jumlah satelit; mungkin fix belum stabil                                            |

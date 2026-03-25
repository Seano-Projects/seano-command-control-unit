# Seano Workspace - ROS 2

Workspace untuk sistem USV (Unmanned Surface Vehicle) berbasis ROS 2 Humble.

## Struktur Paket

- **seano_command** - Command handling untuk kontrol USV
- **seano_logging** - Logging telemetry data
- **seano_mqtt_bridge** - Bridge antara ROS 2 dan MQTT
- **seano_oceanography** - Node sensor oceanography (CTD, ADCP, SBES)
- **seano_startup** - Launch files untuk startup sistem
- **seano_telemetry** - Telemetry data processing

## Build Workspace

```bash
cd /home/seano/Seano_ws
colcon build --symlink-install
```

## Setup Environment

Setiap kali membuka terminal baru, source workspace:

```bash
source /home/seano/Seano_ws/install/setup.bash
```

## Menjalankan Sistem

### Jalankan Sistem Lengkap

Launch semua node sekaligus dengan launch file:

```bash
ros2 launch seano_startup system.launch.py
```

Launch file ini akan menjalankan:
- **MAVROS** - Koneksi ke flight controller (FCU: `/dev/ttyACM0:57600`, GCS: `udp://@0.0.0.0:14550`)
- **Telemetry Node** - Processing data telemetry
- **Telemetry Logger** - Logging telemetry ke file
- **MQTT Bridge** - Bridge untuk komunikasi MQTT

### Jalankan Node Individual

Untuk menjalankan node secara terpisah:

```bash
# Telemetry node
ros2 run seano_telemetry telemetry_node

# Telemetry logger
ros2 run seano_logging telemetry_logger_node

# MQTT bridge
ros2 run seano_mqtt_bridge mqtt_bridge_node

# CTD oceanography sensor
ros2 run seano_oceanography ctd_sensor_node --ros-args --params-file src/seano_startup/config/system.yaml
```

## Menjalankan Seano Oceanography (CTD)

### 1) Build package yang diperlukan

```bash
cd /home/seano/Seano_ws
colcon build --packages-select seano_oceanography seano_startup
```

### 2) Source environment

```bash
source /opt/ros/humble/setup.bash
source /home/seano/Seano_ws/install/setup.bash
```

### 3) Jalankan generator data CTD

```bash
ros2 run seano_oceanography ctd_sensor_node --ros-args --params-file /home/seano/Seano_ws/src/seano_startup/config/system.yaml
```

### 4) Monitor data ROS

```bash
ros2 topic echo /oceanography/ctd
```

### 5) Monitor data MQTT

Format topic CTD:

```text
seano/{vehicle_code}/{sensor_code}/data
```

Contoh topic dari config default:

```text
seano/USV-001/CTD-MIDAS-3000/data
```

Subscribe topic MQTT:

```bash
mosquitto_sub -h mqtt.seano.cloud -p 8883 -u seanomqtt -P 'Seano2025*' --insecure -t 'seano/USV-001/CTD-MIDAS-3000/data'
```

Contoh payload CTD:

```json
{
  "date_time": "2026-03-17T18:30:12+07:00",
  "vehicle_code": "USV-001",
  "sensor_code": "CTD-MIDAS-3000",
  "sensor": "CTD",
  "depth_m": 53.21,
  "pressure_m": 54.27,
  "temperature_c": 26.44,
  "conductivity_ms_cm": 50.84,
  "salinity_psu": 33.12,
  "density_kg_m3": 1020.73,
  "sound_velocity_ms": 1535.47
}
```

## Konfigurasi

File konfigurasi sistem berada di:
```
src/seano_startup/config/system.yaml
```

Parameter CTD yang dipakai:
- `vehicle.id`
- `oceanography.ctd.sensor_code`
- `oceanography.ctd.timezone`
- `oceanography.ctd.publish_topic`
- `oceanography.ctd.publish_rate_hz`
- `oceanography.ctd.max_depth_m`
- `oceanography.ctd.cycle_seconds`

## Debug & Monitoring

Lihat daftar topic yang aktif:
```bash
ros2 topic list
```

Monitor topic tertentu:
```bash
ros2 topic echo /usv/telemetry
```

Lihat info node:
```bash
ros2 node list
ros2 node info /usv/telemetry
```

### Cek Data MQTT

Sistem mengirim telemetry ke MQTT broker setiap 1 detik:

**MQTT Broker:** `mqtt.seano.cloud:8883` (TLS)  
**Topic:** `seano/USV-001/telemetry`  

**Data yang dikirim (JSON):**
```json
{
  "vehicle_code": "USV-001",
  "battery_voltage": 11.5,
  "battery_current": 2.3,
  "battery_percentage": 85,
  "rssi": -65,
  "latitude": -6.2088,
  "longitude": 107.8456,
  "altitude": 10.5,
  "heading": 90.5,
  "armed": true,
  "gps_ok": true,
  "system_status": "OK",
  "mode": "AUTO",
  "speed": 5.2,
  "roll": 15.0,
  "pitch": 3.5,
  "yaw": 90.5,
  "temperature_system": 30.0
}
```

**Field yang dikumpulkan:**
- **vehicle_code** - ID kendaraan (dari config)
- **battery_voltage** - Tegangan baterai (V) dari `/mavros/battery`
- **battery_current** - Arus baterai (A) dari `/mavros/battery`
- **battery_percentage** - Persentase baterai (0-100)
- **rssi** - Signal strength dari `/mavros/radio_status`
- **latitude, longitude, altitude** - Posisi GPS dari `/mavros/global_position/global`
- **heading** - Arah heading (0-360°)
- **armed** - Status armed dari `/mavros/state`
- **gps_ok** - Status GPS fix
- **system_status** - Status koneksi ke FCU (OK/DISCONNECTED)
- **mode** - Flight mode (MANUAL/AUTO/dll)
- **speed** - Kecepatan groundspeed (m/s) dari `/mavros/vfr_hud`
- **roll, pitch, yaw** - Attitude dari IMU
- **temperature_system** - Suhu sistem dari `/mavros/temperature`

Monitor data yang dikirim ke MQTT:
```bash
ros2 topic echo /usv/telemetry
```

## Troubleshooting

### Error: StopIteration saat jalankan node

Jika muncul error `StopIteration` di `importlib_load_entry_point`, berarti package perlu di-rebuild:

```bash
cd /home/seano/Seano_ws
colcon build --symlink-install
source install/setup.bash
```

### Error: /dev/ttyACM0 tidak ditemukan

Error `DeviceError:serial:open: No such file or directory` pada MAVROS menandakan flight controller belum terhubung. Ini normal jika device belum dicolok.

Cek device yang tersedia:
```bash
ls /dev/ttyACM*
ls /dev/ttyUSB*

# Atau lihat device by-id
ls -la /dev/serial/by-id/
```

### Warning: AHRS waiting to be healthy

Ini normal saat flight controller baru boot. Error akan hilang kalau:
- GPS sudah lock (bawa ke outdoor)
- IMU/compass sudah terkalibrasi
- Tunggu 30-60 detik

### Warning: QoS incompatibility

Jika muncul warning tentang QoS compatibility, telemetry node tidak akan menerima data. Sudah diperbaiki dengan QoS BEST_EFFORT untuk sensor topics.

### Rebuild Semua Package

Jika ada masalah setelah edit code:
```bash
cd /home/seano/Seano_ws
rm -rf build/ install/ log/
colcon build --symlink-install
source install/setup.bash
```

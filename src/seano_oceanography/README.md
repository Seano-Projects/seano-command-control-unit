# seano_oceanography

Package sensor oseanografi untuk data CTD dan ADCP.

## Node

- `ctd_sensor_node` — publish data CTD ke ROS2 dan MQTT. GPS-gated: hanya kirim MQTT bila GPS sudah fix.
- `adcp_sensor_node` — publish data ADCP ke ROS2 dan MQTT. GPS-gated: hanya kirim MQTT bila GPS sudah fix.

---

## ROS2 Interface

### Published

| Topic               | Tipe                     | Node               |
| ------------------- | ------------------------ | ------------------ |
| `oceanography/ctd`  | `std_msgs/String` (JSON) | `ctd_sensor_node`  |
| `oceanography/adcp` | `std_msgs/String` (JSON) | `adcp_sensor_node` |

### Subscribed

| Topic                            | Tipe                    | Node      | Keterangan                    |
| -------------------------------- | ----------------------- | --------- | ----------------------------- |
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | CTD, ADCP | Lat, lon, alt, GPS fix status |
| `/mavros/vfr_hud`                | `mavros_msgs/VfrHud`    | ADCP      | Heading kompas (°)            |

---

## MQTT

### Kirim (Publish)

| Topic                                     | Node               | Kondisi kirim             |
| ----------------------------------------- | ------------------ | ------------------------- |
| `seano/{vehicle_code}/{sensor_code}/data` | `ctd_sensor_node`  | GPS fix (`gps_ok = true`) |
| `seano/{vehicle_code}/{sensor_code}/data` | `adcp_sensor_node` | GPS fix (`gps_ok = true`) |

> **GPS Guard:** Selama GPS belum fix, data tetap publish ke ROS topic internal tapi **tidak dikirim ke MQTT**. Ini mencegah data dengan koordinat kosong/default masuk ke cloud.

### Terima (Subscribe)

Tidak ada subscribe MQTT di package ini.

---

## JSON Payload — CTD

Topic MQTT: `seano/USV-001/CTD-MIDAS-3000/data`

```json
{
  "date_time": "2026-05-01T15:41:41.204+07:00",
  "vehicle_code": "USV-001",
  "sensor_code": "CTD-MIDAS-3000",
  "sensor": "CTD",
  "latitude": -6.2000123,
  "longitude": 106.8166789,
  "altitude": 2.5,
  "gps_ok": true,
  "depth_m": 45.2,
  "pressure_m": 46.1,
  "temperature_c": 27.83,
  "conductivity_ms_cm": 52.14,
  "salinity_psu": 33.21,
  "density_kg_m3": 1024.87,
  "sound_velocity_ms": 1523.4
}
```

---

## JSON Payload — ADCP

Topic MQTT: `seano/USV-001/ADCP-WORKHORSE/data`

```json
{
  "date_time": "2026-05-01T15:41:41.204+07:00",
  "vehicle_code": "USV-001",
  "sensor_code": "ADCP-WORKHORSE",
  "sensor": "ADCP",
  "latitude": -6.2000123,
  "longitude": 106.8166789,
  "altitude": 2.5,
  "heading_deg": 124.5,
  "gps_ok": true,
  "ensemble_no": 42,
  "temperature_c": 28.7,
  "v1_ms": -0.052,
  "v2_ms": 0.089,
  "v3_ms": -0.031,
  "v4_ms": 0.047,
  "current_speed_ms": 0.187,
  "current_direction_deg": 145.3,
  "water_depth_m": 87.4
}
```

### Deskripsi Field ADCP

| Field                   | Satuan | Keterangan                                                    |
| ----------------------- | ------ | ------------------------------------------------------------- |
| `heading_deg`           | °      | Heading kapal dari `/mavros/vfr_hud`, 0–360° (North = 0°)     |
| `ensemble_no`           | -      | Nomor ensemble, increment tiap pengukuran (khas Teledyne RDI) |
| `temperature_c`         | °C     | Suhu air permukaan dari transducer ADCP                       |
| `v1_ms`–`v4_ms`         | m/s    | Kecepatan radial tiap beam (Janus 4-beam, sudut 20°)          |
| `current_speed_ms`      | m/s    | Kecepatan arus total — magnitude vektor horizontal            |
| `current_direction_deg` | °      | Arah arus di frame Bumi, 0° = North clockwise                 |
| `water_depth_m`         | m      | Kedalaman air di bawah transducer                             |

---

## Dummy Data Generator — Fisika ADCP

Data simulasi menggunakan model fisika berlapis, bukan `random`. Alur kalkulasi:

### 1. Komponen Arus di Earth Frame

Arus laut dimodelkan sebagai superposisi gelombang tidal & oskilasi:

```
u_east  = A₁·sin(ω₁·t + φ₁) + A₂·sin(ω₂·t + φ₂)   [m/s ke Timur]
v_north = B₁·cos(ω₁·t + φ₃) + B₂·cos(ω₂·t + φ₄)   [m/s ke Utara]
```

### 2. Kecepatan & Arah Arus

$$\text{speed} = \sqrt{u_{east}^2 + v_{north}^2}$$

$$\text{direction} = \text{atan2}(u_{east},\, v_{north}) \pmod{360°}$$

(Konvensi meteorologi: 0° = North, clockwise)

### 3. Rotasi Earth → Instrument Frame

Komponen arus dirotasi menggunakan heading kapal $\psi$:

$$u_{inst} = u_{east}\cos\psi + v_{north}\sin\psi$$
$$v_{inst} = -u_{east}\sin\psi + v_{north}\cos\psi$$

### 4. Proyeksi ke Beam (Janus 4-beam, sudut 20°)

Geometri Teledyne RDI: beam 1&2 resolve sumbu X, beam 3&4 resolve sumbu Y:

$$V_1 = -u_{inst} \cdot \sin(20°) \qquad V_2 = +u_{inst} \cdot \sin(20°)$$
$$V_3 = -v_{inst} \cdot \sin(20°) \qquad V_4 = +v_{inst} \cdot \sin(20°)$$

> Saat sensor fisik terhubung, alurnya **kebalik**: V1–V4 → instrument frame → Earth frame → speed & direction.

---

## Parameter Penting

### CTD

| Parameter                          | Default                          | Keterangan                    |
| ---------------------------------- | -------------------------------- | ----------------------------- |
| `oceanography.ctd.publish_topic`   | `oceanography/ctd`               | ROS topic                     |
| `oceanography.ctd.publish_rate_hz` | `1.0`                            | Frekuensi publish             |
| `oceanography.ctd.sensor_code`     | `CTD-MIDAS-3000`                 | Bagian dari MQTT topic        |
| `oceanography.ctd.gps_topic`       | `/mavros/global_position/global` | Sumber GPS                    |
| `oceanography.ctd.max_depth_m`     | `120.0`                          | Kedalaman maksimum profiling  |
| `oceanography.ctd.cycle_seconds`   | `120.0`                          | Durasi satu siklus dive/climb |

### ADCP

| Parameter                             | Default                          | Keterangan                      |
| ------------------------------------- | -------------------------------- | ------------------------------- |
| `oceanography.adcp.publish_topic`     | `oceanography/adcp`              | ROS topic                       |
| `oceanography.adcp.publish_rate_hz`   | `1.0`                            | Frekuensi publish               |
| `oceanography.adcp.sensor_code`       | `ADCP-WORKHORSE`                 | Bagian dari MQTT topic          |
| `oceanography.adcp.gps_topic`         | `/mavros/global_position/global` | Sumber GPS                      |
| `oceanography.adcp.vfr_topic`         | `/mavros/vfr_hud`                | Sumber heading                  |
| `oceanography.adcp.max_water_depth_m` | `250.0`                          | Kedalaman air maksimum simulasi |

### MQTT (shared)

| Parameter                         | Keterangan                    |
| --------------------------------- | ----------------------------- |
| `mqtt.broker`                     | Alamat broker                 |
| `mqtt.port`                       | Port (default 8883 TLS)       |
| `mqtt.username` / `mqtt.password` | Kredensial                    |
| `mqtt.base_topic`                 | Prefix topic, default `seano` |
| `mqtt.qos`                        | QoS level                     |

---

## Jalankan

### Semua sekaligus (via launch)

```bash
ros2 launch seano_startup system.launch.py
```

### Satu per satu (debug)

```bash
ros2 run seano_oceanography ctd_sensor_node --ros-args --params-file /home/seano/Seano_ws/src/seano_startup/config/system.yaml
ros2 run seano_oceanography adcp_sensor_node --ros-args --params-file /home/seano/Seano_ws/src/seano_startup/config/system.yaml
```

### Cek cepat

```bash
# Cek node jalan
ros2 node list | grep -E 'ctd|adcp'

# Monitor data
ros2 topic echo oceanography/adcp
ros2 topic echo oceanography/ctd

# Cek GPS status
ros2 topic echo /mavros/global_position/global --once
```

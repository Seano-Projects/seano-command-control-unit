# seano_anti_theft

Package keamanan USV (Unmanned Surface Vehicle) yang memantau kondisi kapal secara real-time dan mendeteksi potensi pencurian atau situasi berbahaya. Node ini bekerja dengan membaca data langsung dari MAVROS (flight controller) dan mengirim alert ke web dashboard via MQTT ketika ancaman terdeteksi.

---

## Daftar Isi

- [Fungsi Utama](#fungsi-utama)
- [Input (Subscribe)](#input-subscribe)
- [Output (Publish)](#output-publish)
- [Logika Deteksi](#logika-deteksi)
- [Parameter Konfigurasi](#parameter-konfigurasi)
- [Cara Menjalankan](#cara-menjalankan)
- [Struktur Package](#struktur-package)
- [Troubleshooting](#troubleshooting)

---

## Fungsi Utama

Node `anti_theft_node` memantau 3 kondisi bahaya:

| Kondisi | Trigger | Aksi |
|---|---|---|
| **GEOFENCE BREACH** | Kapal bergerak > 10 m dari posisi saat RC hilang | Kirim alert + paksa mode RTL ke flight controller |
| **BOAT FLIPPED** | Kemiringan (tilt) > 10° selama > 7 detik berturut-turut | Kirim alert |
| **TOWING DETECTED** | Mode AUTO/GUIDED tapi groundspeed melebihi batas target + margin | Kirim alert |

**Kondisi security aktif** — sistem keamanan baru aktif saat **sinyal RC hilang** (`RC Channel 3 PWM < 975`). Saat RC aktif, sistem dalam mode standby dan tidak mendeteksi ancaman.

---

## Input (Subscribe)

### MAVROS Topics

| Topic | Tipe | Keterangan |
|---|---|---|
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | Posisi GPS (lat, lon, alt, fix status) |
| `/mavros/imu/data` | `sensor_msgs/Imu` | Data IMU — quaternion dikonversi ke roll/pitch untuk hitung tilt |
| `/mavros/vfr_hud` | `mavros_msgs/VfrHud` | Groundspeed dan heading kapal |
| `/mavros/state` | `mavros_msgs/State` | Mode flight controller (AUTO, GUIDED, RTL, MANUAL, dll) |
| `/mavros/rc/in` | `mavros_msgs/RCIn` | Sinyal RC — channel 3 (PWM) untuk deteksi kehilangan RC |

### MAVROS Service (Call)

| Service | Tipe | Kapan dipanggil |
|---|---|---|
| `/mavros/set_mode` | `mavros_msgs/SetMode` | Paksa mode RTL saat GEOFENCE BREACH terdeteksi |

---

## Output (Publish)

### ROS Topics

| Topic | Tipe | Isi | Kapan dikirim |
|---|---|---|---|
| `anti_theft/alert` | `std_msgs/String` | String alarm: `GEOFENCE BREACH`, `BOAT FLIPPED`, atau `TOWING DETECTED` | Hanya saat alarm aktif |

### MQTT Topics

| Topic | Format | Kapan dikirim |
|---|---|---|
| `seano/{vehicle_id}/anti_theft/alert` | JSON | Hanya saat alarm aktif |

**Contoh payload MQTT alert:**
```json
{
  "vehicle_id": "USV-001",
  "alert": "GEOFENCE BREACH"
}
```

---

## Logika Deteksi

### Alur Keamanan

```
RC Channel 3 PWM < 975 (RC hilang)
        │
        ▼
  SECURITY ACTIVE
  (simpan home location saat ini)
        │
        ├─── Mode AUTO/GUIDED? ──► Cek kecepatan
        │                              groundspeed > target + margin?
        │                              └──► alarm: TOWING DETECTED
        │
        └─── Mode lainnya? ──► Cek geofence
                                   drift > 10m?
                                   └──► alarm: GEOFENCE BREACH
                                        + kirim RTL ke FC
                               Cek tilt
                                   tilt > 10° selama > 7 detik?
                                   └──► alarm: BOAT FLIPPED

RC Channel 3 PWM >= 975 (RC aktif)
        │
        ▼
  SECURITY STANDBY (tidak ada deteksi)
```

### Penghitungan Tilt

Tilt dihitung dari quaternion IMU:

$$\text{tilt} = \sqrt{\text{roll}^2 + \text{pitch}^2}$$

Threshold default: **10°** selama **7 detik** berturut-turut.

### Penghitungan Drift (Geofence)

Jarak kapal dari home location dihitung dengan Haversine formula (akurat untuk jarak pendek di permukaan bumi). Threshold default: **10 meter**.

---

## Parameter Konfigurasi

Konfigurasi ada di `~/Seano_ws/src/seano_startup/config/system.yaml`:

```yaml
anti_theft:
  loop_rate_hz: 1.0           # Frekuensi loop utama (Hz)
  alert_topic: anti_theft/alert
  target_speed_mps: 1.0       # Kecepatan target misi (m/s)

  mavros:
    gps_topic: /mavros/global_position/global
    imu_topic: /mavros/imu/data
    vfr_topic: /mavros/vfr_hud
    state_topic: /mavros/state
    rc_in_topic: /mavros/rc/in
    set_mode_service: /mavros/set_mode

  mqtt_enabled: true
  geofence_limit: 10.0        # Batas geofence dalam meter
  crit_tilt_deg: 10.0         # Threshold tilt kritis (derajat)
  tilt_confirm_time: 7.0      # Durasi tilt harus terjadi sebelum alarm (detik)
  rc_failsafe_pwm: 975        # Nilai PWM RC di bawah ini dianggap RC hilang
  mission_speed_margin: 1.5   # Margin kecepatan di atas target (m/s) sebelum alarm towing
```

**Parameter MQTT** (shared di system.yaml):

```yaml
mqtt:
  broker: <broker_host>
  port: 8883
  username: <username>
  password: <password>
  base_topic: seano
  qos: 1
  use_tls: true
  tls_insecure: true
```

---

## Cara Menjalankan

### Prasyarat

```bash
# Pastikan workspace sudah di-build
cd ~/Seano_ws
colcon build --base-paths src --packages-select seano_anti_theft

# Source workspace
source ~/Seano_ws/install/setup.bash
```

MAVROS harus sudah berjalan dengan koneksi ke flight controller.

---

### 1. Jalankan dengan Launch File (Rekomendasi)

```bash
source ~/Seano_ws/install/setup.bash
ros2 launch seano_anti_theft anti_theft.launch.py
```

Launch file otomatis membaca `system.yaml` sebagai parameter.

---

### 2. Jalankan Node Langsung

```bash
source ~/Seano_ws/install/setup.bash
ros2 run seano_anti_theft anti_theft_node \
  --ros-args --params-file ~/Seano_ws/src/seano_startup/config/system.yaml
```

---

### 3. Jalankan dengan Parameter Override

```bash
source ~/Seano_ws/install/setup.bash
ros2 run seano_anti_theft anti_theft_node \
  --ros-args \
  --params-file ~/Seano_ws/src/seano_startup/config/system.yaml \
  -p anti_theft.geofence_limit:=20.0 \
  -p anti_theft.mqtt_enabled:=false
```

---

### 4. Jalankan Bersama Sistem (via seano_startup)

Anti-theft node dijalankan otomatis ketika sistem SEANO distart secara penuh. Cukup jalankan:

```bash
bash ~/Seano_ws/start_seano.sh
```

---

## Monitoring & Debug

### Pantau alert topic

```bash
source ~/Seano_ws/install/setup.bash
ros2 topic echo anti_theft/alert
```

### Pantau data RC channel

```bash
ros2 topic echo /mavros/rc/in
# Channel index 2 (ke-3) adalah RC3 — throttle/failsafe channel
```

### Pantau GPS dan IMU

```bash
ros2 topic echo /mavros/global_position/global
ros2 topic echo /mavros/imu/data
```

### Cek node aktif

```bash
ros2 node list | grep anti_theft
ros2 node info /anti_theft_node
```

---

## Struktur Package

```
seano_anti_theft/
├── launch/
│   └── anti_theft.launch.py        # Launch file utama
├── seano_anti_theft/
│   ├── __init__.py
│   └── anti_theft_node.py          # Node utama
├── package.xml
├── setup.py
└── README.md
```

---

## Troubleshooting

| Masalah | Penyebab | Solusi |
|---|---|---|
| `Menunggu GPS fix / posisi...` terus menerus | MAVROS belum konek ke FC atau GPS belum fix | Pastikan MAVROS berjalan dan FC mendapat sinyal GPS |
| Node tidak muncul di `ros2 node list` | Package belum di-build atau belum di-source | `colcon build` lalu `source install/setup.bash` |
| MQTT tidak terkirim | `mqtt_enabled: false` atau broker tidak bisa diakses | Cek koneksi ke broker, cek TLS setting |
| Geofence tidak aktif meski RC hilang | `rc_failsafe_pwm` tidak sesuai nilai failsafe RC | Cek nilai RC3 saat RC dimatikan: `ros2 topic echo /mavros/rc/in`, sesuaikan `rc_failsafe_pwm` |
| RTL tidak terpicu saat geofence breach | MAVROS service `/mavros/set_mode` tidak tersedia | Pastikan MAVROS berjalan dan FC terhubung |

# Issue: MAVROS Disconnected — ARM / FORCE_ARM / Mode Change Gagal

**Tanggal ditemukan:** 2026-06-13  
**Status:** Resolved  
**Komponen:** `seano_command`, `seano_startup`, MAVROS, CUAV-X7

---

## Gejala

- `ARM` selalu gagal: `Pre-arm check failed: GPS no fix`
- `FORCE_ARM` selalu gagal: `FORCE_ARM failed: armed state not reached` (~4 detik)
- `AUTO` / mode change gagal: `Mode change to AUTO failed`
- Tidak ada output di `/mavros/statustext/recv`
- Telemetry menunjukkan `DISCONNECTED` sepanjang sesi

---

## Alur Debugging

### Step 1 — Cek log command

Lihat pola durasi di `ros_log/command/command_log_YYYYMMDD.csv`:

| Pattern | Artinya |
|---|---|
| ARM gagal < 10ms, `mavlink_sent_timestamp` kosong | GPS pre-arm check blokir di software, MAVROS tidak disentuh |
| FORCE_ARM gagal tepat ~4000ms | `_confirm_arm_state` timeout — MAVROS menerima command tapi FCU tidak arm |
| FORCE_ARM gagal ~9000ms | 5s `wait_for_mavros_service` retry + 4s timeout — service sempat unavailable |
| Mode change gagal < 30ms | MAVROS disconnect atau FCU tolak (misal AUTO butuh GPS) |

### Step 2 — Cek MAVROS state

```bash
ros2 topic echo /mavros/state --once
```

- `connected: false` + `mode: ''` → MAVROS tidak dapat heartbeat dari FCU
- `connected: true` → lanjut ke Step 4 (cek FCU side)

### Step 3 — Cek berapa mavros_node yang jalan

```bash
ps aux | grep mavros_node | grep -v grep
```

**Harus hanya ada 1 proses.** Kalau ada 2 atau lebih → duplicate MAVROS berebut serial port.

**Penyebab:** `system.launch.py` spawn MAVROS sendiri sementara `mavros.service` (systemd) juga jalan.  
**Fix:** Hapus `mavros_launch` dari `system.launch.py` — biarkan `mavros.service` yang handle.

```bash
# Matikan proses duplicate (ganti PID sesuai hasil ps)
sudo kill <PID_duplicate>
```

MAVROS yang benar akan auto-reconnect dalam 30 detik.

### Step 4 — Cek port serial FCU

```bash
ls /dev/ttyACM* /dev/ttyUSB* /dev/ttyTHS* 2>/dev/null
```

Kalau port tidak ada:

```bash
lsusb | grep -i "1209\|ArduPilot\|CUAV"
```

- Device muncul di `lsusb` tapi tidak ada `ttyACM` → cek permission: `sudo chmod 666 /dev/ttyACM0`
- Device tidak muncul di `lsusb` sama sekali → **hardware issue** (lanjut Step 5)

### Step 5 — Hardware check

1. Cek LED FCU — masih nyala?
2. **Ganti kabel USB** — kabel rusak bagian dalam sering terlihat normal tapi tidak bisa transfer data
3. Coba port USB berbeda di komputer
4. Cabut semua power ke FCU, tunggu 10 detik, colok ulang
5. Setelah colok: `lsusb` harus menampilkan `ID 1209:5740 Generic CUAV-X7`

---

## Root Cause yang Ditemukan (2026-06-13)

### Bug 1 — Dual MAVROS (seano_startup)

`system.launch.py` include `mavros apm.launch` sementara `mavros.service` di systemd juga running. Dua `mavros_node` berebut `/dev/ttyACM0` → satu connect, satu gagal terus. Kalau keduanya sempat connect bersamaan → MAVLink serial communication corrupt → semua operasi (ARM, mode change, waypoint) jadi sangat lambat atau gagal total.

**Fix:** Hapus `mavros_launch` dan `fcu_url`/`gcs_url` dari `system.launch.py`.

### Bug 2 — `response.result` vs `response.success` (command_node)

```python
# Sebelum (salah)
result = int(getattr(response, 'result', -1))
success = (result == 0)

# Sesudah (benar)
success = bool(getattr(response, 'success', False))
```

Ketika MAVROS tidak connect ke FCU, `CommandLong` service bisa return `result=0` (default `uint8`) meski `success=False`. Akibatnya FORCE_ARM selalu masuk `_confirm_arm_state` dan buang waktu 4 detik sebelum akhirnya timeout, dengan pesan error yang menyesatkan (`armed state not reached` bukan `MAVROS service unavailable`).

### Bug 3 — Duplicate waypoint handler (seano_command + seano_mission)

`waypoint_node` (seano_command) dan `mission_node` (seano_mission) subscribe ke MQTT topic yang sama:

```
seano/USV-001/waypoint
```

Keduanya memanggil `/mavros/mission/push` secara bersamaan saat ada waypoint masuk. MAVLink mission upload protocol adalah stateful handshake — dua concurrent call mengkorupsi state machine di MAVROS, menyebabkan serial communication hang dan semua operasi berikutnya (ARM, mode change) ikut gagal atau sangat lambat.

**Tanda di log:** Waypoint upload tiba-tiba butuh 5000ms+ (normal < 500ms), diikuti mode change yang butuh 20-30 detik.

### Root Cause Akhir (hari ini)

Kabel USB CUAV-X7 putus secara fisik. CUAV-X7 tidak muncul di `lsusb`. **Solusi: ganti kabel USB.**

---

## Checklist Debug Cepat

```
[ ] ros2 topic echo /mavros/state --once → connected?
[ ] ps aux | grep mavros_node → hanya 1 proses?
[ ] ls /dev/ttyACM* → port ada?
[ ] lsusb | grep 1209 → FCU terdeteksi OS?
[ ] LED FCU nyala?
[ ] Kabel USB OK? (coba ganti)
```

---

## File Terkait

- `src/seano_startup/launch/system.launch.py` — hapus duplicate mavros launch
- `src/seano_command/seano_command/command_node.py` — fix `response.success` check
- `src/seano_command/seano_command/waypoint_node.py` — duplicate topic dengan mission_node
- `src/seano_mission/seano_mission/mission_node.py` — duplicate topic dengan waypoint_node

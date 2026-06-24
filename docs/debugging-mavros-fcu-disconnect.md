# Debugging: MAVROS Tidak Konek ke Flight Controller

## Gejala

Command di `ros_log/command/command_log_*.csv` gagal terus dengan pesan:

- `Pre-arm check failed: GPS no fix`
- `MAVROS service unavailable`
- `Mode change to ... failed`

## Cek Cepat: Apakah MAVROS Konek ke FCU?

```bash
ros2 topic echo /mavros/state --once
```

Kalau `connected: false`, MAVROS belum berhasil buka koneksi serial ke flight controller.

## Cek Log MAVROS

```bash
sudo systemctl status mavros.service --no-pager
```

Kalau muncul error berulang seperti:

```
DeviceError:serial:open: No such file or directory
```

berarti device path (`/dev/ttyACMx`) di config tidak ada / berubah.

## Cek Device Serial yang Tersedia

```bash
ls -la /dev/ttyACM*
```

Cocokkan dengan flight controller pakai `udevadm` (cari `ID_MODEL`/`ID_VENDOR` ArduPilot/CUAV):

```bash
udevadm info -q property -n /dev/ttyACM1 | grep -E "ID_MODEL|ID_VENDOR|ID_SERIAL"
```

Port FCU bisa berpindah nomor (misal `ttyACM0` -> `ttyACM1`) setiap kali USB di-replug atau board reboot.

## Cara Perbaiki: Override `SEANO_FCU_URL` di systemd

`mavros.service` default-nya pakai `Environment=SEANO_FCU_URL=/dev/ttyACM0:115200` (lihat `systemctl cat mavros.service`). Untuk override tanpa edit file unit asli, pakai drop-in override:

```bash
sudo systemctl edit mavros.service
```

Di editor (nano), **isi bagian kosong di atas** (sebelum baris `### Lines below this comment will be discarded`) dengan:

```ini
[Service]
Environment=SEANO_FCU_URL=/dev/ttyACM1:115200
```

> Penting: jangan edit baris yang sudah berupa komentar (`#...`) di bawah marker tersebut — itu cuma preview file asli dan akan dibuang, perubahan di sana **tidak** akan tersimpan.

Save (`Ctrl+O` lalu Enter) dan keluar (`Ctrl+X`).

## Terapkan Perubahan

```bash
sudo systemctl daemon-reload
sudo systemctl restart mavros.service
sleep 4
sudo systemctl status mavros.service --no-pager | head -15
```

## Verifikasi Sudah Konek

```bash
ros2 topic echo /mavros/state --once
```

Harus `connected: true` sekarang.

## Catatan Lain

- File override tersimpan di `/etc/systemd/system/mavros.service.d/override.conf`.
- Ada juga `mavlink-router-main.conf` di `src/seano_anti_theft/config/` yang punya `Device=/dev/ttyACM0` hardcoded — kalau pakai mavlink-router (bukan mavros langsung ke serial), path di file ini juga perlu disamakan.
- Pertimbangkan bikin udev rule symlink stabil (misal `/dev/ttyFCU`) berdasarkan `ID_SERIAL` device, supaya tidak perlu ganti config manual setiap port berubah nomor.

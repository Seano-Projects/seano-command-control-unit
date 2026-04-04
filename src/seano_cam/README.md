# seano_cam

Package ini menangani kamera USB di USV: deteksi device, publish frame ke ROS2 topic, viewer lokal, dan streaming RTMP.

Catatan integrasi terbaru: untuk menghindari bentrok akses device kamera, mode yang direkomendasikan adalah
`seano_vision` sebagai sumber kamera + deteksi, sedangkan `seano_cam` fokus untuk streaming RTMP.

## Node

- `camera_node`: deteksi kamera, publish status kamera, publish frame kamera.
- `camera_viewer`: subscribe topic image lalu tampilkan dengan OpenCV window.
- `rtmp_streamer`: subscribe topic image lalu kirim stream ke server RTMP dengan FFmpeg.

## ROS2 Interface

### Published

- `/seano/{vehicle_id}/camera/status` (`std_msgs/String`) oleh `camera_node`
- `/seano/{vehicle_id}/camera/image` (`sensor_msgs/Image`) oleh `camera_node` jika `camera.enable_publish=true`

### Subscribed

- Topic image kamera (default `/seano/SEANO001/camera/image`) oleh `camera_viewer`
- Topic image kamera (default `/camera/image_annotated`) oleh `rtmp_streamer`

## MQTT

Tidak ada koneksi MQTT langsung di package ini.

## Parameter Penting

- `vehicle.id`
- `camera.check_interval`
- `camera.device`
- `camera.enable_display`
- `camera.enable_publish`
- `camera.fps`
- `rtmp.url`
- `rtmp.width`, `rtmp.height`, `rtmp.fps`, `rtmp.bitrate`, `rtmp.preset`
- `camera.topic` (untuk `rtmp_streamer`)

Rekomendasi integrasi:
- Jalankan `camera_node` hanya jika tidak memakai kamera dari `seano_vision`.
- Jika `seano_vision` aktif, gunakan `camera.topic=/camera/image_annotated` agar stream RTMP menampilkan hasil deteksi.

## Catatan: vehicle.id dengan Tanda Hubung (-)

`vehicle.id` seperti `USV-001` tidak valid langsung dipakai sebagai nama ROS2 topic karena karakter `-` dilarang.
`camera_node` sudah otomatis menggantinya menjadi `_` untuk nama topic (contoh: `USV-001` → `USV_001`).
Pastikan package sudah di-build ulang setelah perubahan ini (lihat langkah build di bawah).

## Setup PyTorch CUDA untuk Jetson (JetPack 6.x / CUDA 12.6)

Wajib dilakukan sekali agar YOLO bisa jalan di GPU Jetson.

### Step 1 — Uninstall torch versi salah

```bash
pip3 uninstall torch torchvision -y
```

### Step 2 — Install torch versi Jetson (CUDA 12.6)

Coba cara ini dulu:

```bash
pip3 install torch torchvision --extra-index-url https://pypi.jetson-ai-lab.dev/jp6/cu126
```

Kalau DNS gagal, download manual di PC lalu SCP ke Jetson:
- Download dari: https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048
- Pilih file `torch-2.x.x-cp310-cp310-linux_aarch64.whl` untuk JetPack 6 / CUDA 12.6

```bash
# Di PC — copy wheel ke Jetson
scp torch-*.whl seano@<ip-jetson>:~/

# Di Jetson — install
pip3 install ~/torch-*.whl
```

### Step 3 — Verifikasi

```bash
python3 -c "import torch; print(torch.__version__); print(torch.cuda.is_available())"
```

Output yang benar:
```
2.x.x+cu126
True
```

---

## Cara Menjalankan

### Step 1 — Build package

Wajib dilakukan setelah clone pertama atau ada perubahan kode:

```bash
cd ~/Seano_ws
colcon build --packages-select seano_cam
source ~/Seano_ws/install/setup.bash
```

### Step 2 — Pilih mode jalankan

#### Opsi A: Streaming kamera saja (tanpa YOLO)

```bash
ros2 launch seano_cam cam.launch.py
```

#### Opsi B: Streaming dengan YOLO detection (GPU)

```bash
ros2 launch seano_cam vision_stream.launch.py
```

Node yang dijalankan:
- `camera_hp` — ambil frame dari kamera USB (seano_vision)
- `detector_node` — deteksi objek dengan YOLOv8n di GPU
- `rtmp_streamer` — push stream annotated ke RTMP server

#### Opsi D: Jalan bareng semua package (via seano_startup)

```bash
ros2 launch seano_startup system.launch.py
```

#### Opsi E: Jalan manual satu per satu (untuk debug)

Buka terminal terpisah untuk masing-masing node:

**Terminal 1 — camera_node** (sumber frame dari kamera USB):
```bash
ros2 run seano_cam camera_node --ros-args --params-file ~/Seano_ws/src/seano_startup/config/system.yaml
```

**Terminal 2 — rtmp_streamer** (kirim stream ke RTMP server):
```bash
ros2 run seano_cam rtmp_streamer --ros-args --params-file ~/Seano_ws/src/seano_startup/config/system.yaml
```

**Terminal 3 — camera_viewer** (opsional, preview lokal via OpenCV window):
```bash
ros2 run seano_cam camera_viewer
```

> **Catatan**: Jika `seano_vision` aktif, **tidak perlu** jalankan `camera_node` lagi. Cukup jalankan `rtmp_streamer` yang subscribe ke `/camera/image_annotated` (frame hasil deteksi).

### Step 3 — Verifikasi stream aktif

```bash
# Cek node rtmp_streamer jalan
ros2 node list | grep rtmp

# Cek topic camera ada frame
ros2 topic list | grep camera
ros2 topic hz /camera/image_annotated

# Diagnostic lengkap (cek RTMP server reachable, streamer jalan, dsb)
cd ~/Seano_ws/src/seano_cam && ./check_stream.sh
```

### Step 4 — Akses di Web

RTMP stream dari `rtmp_streamer` dikirim ke:
```
rtmp://72.61.141.126:1935/live/usv-001
```

Jika RTMP server sudah dikonfigurasi untuk HLS, akses di web player via:
```
http://72.61.141.126:8080/hls/usv-001/index.m3u8
```

URL RTMP dan HLS bisa diubah di `~/Seano_ws/src/seano_startup/config/system.yaml` bagian `rtmp.url`.

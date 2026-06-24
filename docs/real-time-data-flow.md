# Real-time Data Flow (MQTT vs API)

Dokumen ini khusus menjelaskan alur real-time saja, terpisah dari README utama.

## 1) Alur MQTT (USV -> Backend -> Web)

USV publish ke broker MQTT, backend subscribe, simpan ke DB, lalu broadcast ke WebSocket.

**Topik yang dikirim USV (dibaca backend):**

```
seano/{vehicle_code}/telemetry                # vehicle log
seano/{vehicle_code}/{sensor_code}/data      # sensor log
seano/{vehicle_code}/raw                      # raw log (text/JSON)
seano/{vehicle_code}/battery                  # battery status
seano/{vehicle_code}/status                   # LWT online/offline
seano/{vehicle_code}/mission/waypoint_reached # mission progress
seano/{vehicle_code}/antitheft/alert          # alert
seano/{vehicle_code}/failsafe/alert           # alert
seano/{vehicle_code}/command/response         # command response
seano/{vehicle_code}/waypoint/response        # waypoint upload response
```

**WebSocket stream yang menerima hasilnya (ke web):**

```
WS /ws/logs     # vehicle_log, sensor_log, raw_log, command_log, waypoint_log, battery, vehicle_status
WS /ws/alerts   # alert + alert_update
WS /ws/missions # mission_progress + mission_update
```

**Contoh payload MQTT (USV -> backend):**

Topic: `seano/USV-001/telemetry`

```json
{
  "vehicle_code": "USV-001",
  "mission_code": "MSN-a1b2c3d4",
  "battery_voltage": 12.5,
  "battery_current": 2.3,
  "battery_percentage": 85.5,
  "rssi": -65,
  "mode": "AUTO",
  "latitude": -6.2088,
  "longitude": 106.8456,
  "altitude": 10.5,
  "heading": 90.5,
  "armed": true,
  "gps_ok": true,
  "system_status": "OK",
  "speed": 5.2,
  "roll": 0.5,
  "pitch": 1.2,
  "yaw": 90.5,
  "temperature_system": "Normal"
}
```

Topic: `seano/USV-001/CTD-MIDAS-01/data`

```json
{
  "date_time": "2025-01-01T08:30:00Z",
  "vehicle_code": "USV-001",
  "sensor_code": "CTD-MIDAS-01",
  "depth": 25.4,
  "pressure": 2.53,
  "temperature": 27.6,
  "conductivity": 53.2,
  "salinity": 33.8,
  "density": 1024.5
}
```

Topic: `seano/USV-001/raw`

```json
{
  "vehicle_code": "USV-001",
  "logs": "System started successfully"
}
```

Topic: `seano/USV-001/battery`

```json
{
  "vehicle_code": "USV-001",
  "battery_id": 1,
  "percentage": 85.5,
  "voltage": 12.5,
  "current": 2.3,
  "temperature": 32.1,
  "status": "Normal",
  "cell_voltages": [4.1, 4.1, 4.1],
  "cell_count": 3
}
```

Topic: `seano/USV-001/status`

```json
{
  "vehicle_code": "USV-001",
  "status": "online",
  "timestamp": "2025-01-01T08:30:05Z"
}
```

Topic: `seano/USV-001/mission/waypoint_reached`

```json
{
  "vehicle_id": "USV-001",
  "event": "waypoint_reached",
  "wp_seq": 3,
  "total": 12,
  "remaining": 9
}
```

Topic: `seano/USV-001/command/response`

```json
{
  "status": "success",
  "message": "Mode changed to AUTO",
  "vehicle_id": "USV-001"
}
```

Topic: `seano/USV-001/waypoint/response`

```json
{
  "status": "success",
  "message": "Upload 12 waypoint berhasil",
  "vehicle_id": "USV-001"
}
```

Topic: `seano/USV-001/antitheft/alert`

```json
{
  "vehicle_code": "USV-001",
  "severity": "warning",
  "message": "Unauthorized movement detected",
  "source": "USV",
  "latitude": -6.2088,
  "longitude": 106.8456
}
```

Topic: `seano/USV-001/failsafe/alert`

```json
{
  "vehicle_code": "USV-001",
  "severity": "critical",
  "message": "GPS lost, switching to RTL",
  "source": "USV",
  "latitude": -6.2088,
  "longitude": 106.8456
}
```

## 2) Alur API (USV -> Backend -> Web)

Jika USV tidak bisa MQTT, data dikirim via REST API (JWT atau API key), lalu backend menyimpan dan broadcast ke WebSocket.

**Endpoint yang bisa dikirim data dari USV (base URL: https://api.seano.cloud/):**

```
POST https://api.seano.cloud/vehicle-logs
POST https://api.seano.cloud/vehicle-status
POST https://api.seano.cloud/vehicle-batteries
POST https://api.seano.cloud/sensor-logs
POST https://api.seano.cloud/raw-logs
POST https://api.seano.cloud/command-logs
POST https://api.seano.cloud/command-acks
POST https://api.seano.cloud/waypoint-logs
POST https://api.seano.cloud/alerts
PUT https://api.seano.cloud/missions/{id}/progress
POST https://api.seano.cloud/missions/waypoint-reached
POST https://api.seano.cloud/waypoint-acks
GET https://api.seano.cloud/commands/pending
GET https://api.seano.cloud/missions/pending-upload
POST https://api.seano.cloud/control/{vehicle_code}/command
```

**Autentikasi (pilih salah satu):**

- JWT: `Authorization: Bearer <access_token>`
- API key per kendaraan: `X-API-Key: <vehicle_api_key>`

Catatan: jika memakai API key, pastikan request body berisi `vehicle_code` atau `vehicle_id`.

**Generate API key per kendaraan (JWT required):**

```
POST https://api.seano.cloud/vehicles/{vehicle_id}/api-key
```

Response:

```json
{
  "vehicle_id": 1,
  "vehicle_code": "USV-001",
  "api_key": "usv_..."
}
```

**Catatan request body:**

- `POST /vehicle-logs` bisa pakai `vehicle_id` atau `vehicle_code`.
- `POST /vehicle-status` bisa pakai `vehicle_id` atau `vehicle_code` (status: online/offline).
- `POST /vehicle-batteries` bisa pakai `vehicle_id` atau `vehicle_code`.
- `POST /sensor-logs` bisa pakai `vehicle_id`/`vehicle_code` dan `sensor_id`/`sensor_code`.
- `POST /command-logs` dan `POST /waypoint-logs` memakai `vehicle_code` (vehicle_id opsional).
- `POST /command-acks` menerima ACK command dari USV (status: ok/error/success/failed/timeout).
- `POST /alerts` bisa pakai `vehicle_id` atau `vehicle_code`.
- `POST /raw-logs` bisa pakai `vehicle_code` (opsional) + `logs`.
- `POST /missions/waypoint-reached` memakai payload seperti MQTT `mission/waypoint_reached`.
- `POST /control/{vehicle_code}/command` memakai JWT (admin/operator). Jika MQTT nonaktif, command disimpan `pending` untuk dipolling USV.
- `GET /commands/pending` dipakai USV untuk polling command yang `pending`.
- `GET /missions/pending-upload` dipakai USV untuk polling upload mission/waypoint.
- `POST /waypoint-acks` menerima ACK upload mission dari USV (status: ok/error/success/failed/timeout).

**Contoh payload API (USV -> backend):**

```json
POST https://api.seano.cloud/sensor-logs
{
  "vehicle_code": "USV-001",
  "sensor_code": "CTD-MIDAS-01",
  "data": "{\"temperature\":25.5,\"pressure\":1013}"
}
```

```json
POST https://api.seano.cloud/vehicle-logs
{
  "vehicle_code": "USV-001",
  "mission_code": "MSN-a1b2c3d4",
  "battery_voltage": 12.5,
  "battery_current": 2.3,
  "battery_percentage": 85.5,
  "rssi": -65,
  "mode": "AUTO",
  "latitude": -6.2088,
  "longitude": 106.8456,
  "altitude": 10.5,
  "heading": 90.5,
  "armed": true,
  "gps_ok": true,
  "system_status": "OK",
  "speed": 5.2,
  "roll": 0.5,
  "pitch": 1.2,
  "yaw": 90.5,
  "temperature_system": "Normal"
}
```

```json
POST https://api.seano.cloud/vehicle-status
{
  "vehicle_code": "USV-001",
  "status": "online",
  "timestamp": "2025-01-01T08:30:05Z"
}
```

```json
POST https://api.seano.cloud/vehicle-batteries
{
  "vehicle_code": "USV-001",
  "battery_id": 1,
  "percentage": 85.5,
  "voltage": 12.5,
  "current": 2.3,
  "temperature": 32.1,
  "status": "Normal",
  "cell_voltages": [4.1, 4.1, 4.1],
  "cell_count": 3,
  "timestamp": "2025-01-01T08:30:05Z"
}
```

```json
POST https://api.seano.cloud/raw-logs
{
  "vehicle_code": "USV-001",
  "logs": "System started successfully"
}
```

```json
POST https://api.seano.cloud/command-logs
{
  "vehicle_code": "USV-001",
  "command": "AUTO",
  "status": "pending",
  "message": "Requested AUTO mode",
  "initiated_at": "2025-01-01T08:31:00Z"
}
```

```json
POST https://api.seano.cloud/command-acks
{
  "vehicle_code": "USV-001",
  "request_id": "c2e5b7f6-9d2e-4b26-8b0c-2d9802b12b9a",
  "command": "AUTO",
  "status": "ok",
  "message": "Mode set to AUTO",
  "timestamp": "2025-01-01T08:31:02Z"
}
```

```json
POST https://api.seano.cloud/waypoint-logs
{
  "vehicle_code": "USV-001",
  "mission_id": 12,
  "mission_name": "Survey Area A",
  "waypoint_count": 12,
  "status": "pending",
  "message": "Uploading mission",
  "initiated_at": "2025-01-01T08:31:30Z"
}
```

```json
POST https://api.seano.cloud/missions/waypoint-reached
{
  "vehicle_code": "USV-001",
  "event": "waypoint_reached",
  "wp_seq": 3,
  "total": 12,
  "remaining": 9,
  "timestamp": "2025-01-01T08:35:00Z"
}
```

```json
POST https://api.seano.cloud/alerts
{
  "vehicle_code": "USV-001",
  "severity": "critical",
  "alert_type": "failsafe",
  "message": "GPS lost, switching to RTL",
  "latitude": -6.2088,
  "longitude": 106.8456,
  "source": "USV"
}
```

````json
PUT https://api.seano.cloud/missions/12/progress
{
  "progress": 25.0,
  "energy_consumed": 1.2,
  "time_elapsed": 600,
  "current_waypoint": 3,
  "completed_waypoint": 3,
  "status": "Ongoing",
  "timestamp": "2025-01-01T08:35:00Z"
}

```json
GET https://api.seano.cloud/commands/pending?vehicle_code=USV-001&limit=1
{
  "data": [
    {
      "id": 123,
      "vehicle_id": 18,
      "vehicle_code": "USV-001",
      "command": "AUTO",
      "status": "pending",
      "message": "queued",
      "initiated_at": "2025-01-01T08:31:00Z",
      "created_at": "2025-01-01T08:31:00Z"
    }
  ],
  "count": 1
}
````

```json
POST https://api.seano.cloud/waypoint-acks
{
  "vehicle_code": "USV-001",
  "waypoint_log_id": 55,
  "status": "ok",
  "message": "Uploaded",
  "timestamp": "2025-01-01T08:31:40Z"
}
```

```

```

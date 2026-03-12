## SEANO Failsafe System

Package ini menyediakan sistem failsafe untuk SEANO dengan 2 trigger utama:
1. **Battery Critical** - Monitor tegangan & arus dari ESP32 via serial
2. **Communication Loss** - Monitor WiFi, GSM, dan Ethernet (trigger jika SEMUA link down)

## Nodes

### 1. seano_battery
Node untuk monitoring battery dari ESP32 via UART (serial communication).

**Data dari ESP32:**
- Format JSON: `{"voltage": 12.5, "current": 2.3}`
- Atau format simple: `V:12.5,A:2.3`

**Published Topics:**
- `/seano/battery/voltage` (Float32): Tegangan battery (Volt)
- `/seano/battery/current` (Float32): Arus battery (Ampere)
- `/seano/battery/percentage` (Float32): Persentase battery
- `/seano/battery/power` (Float32): Daya (Watt)
- `/seano/battery/status` (String): Status (normal, low, critical, full)
- `/seano/battery/low_alert` (Bool): Alert ketika voltage critical

**Parameters:**
- `failsafe.battery.serial_port` (string, default: /dev/ttyTHS0): Port serial ESP32
- `failsafe.battery.baudrate` (int, default: 115200): Baudrate serial
- `failsafe.battery.check_interval` (float, default: 1.0): Interval publish
- `failsafe.battery.min_voltage` (float, default: 10.5): Voltage minimum (0%)
- `failsafe.battery.max_voltage` (float, default: 12.6): Voltage maximum (100%)
- `failsafe.battery.low_voltage_threshold` (float, default: 11.1): Threshold low voltage
- `failsafe.battery.critical_voltage_threshold` (float, default: 10.8): Threshold critical

### 2. seano_communication_monitor
Node untuk monitoring kekuatan sinyal komunikasi dari **WiFi, GSM, dan Ethernet**.

**Published Topics:**

**Aggregate:**
- `/seano/communication/status` (String): Overall status (all_down, ethernet_active, wifi_good, wifi_weak, gsm_good, gsm_weak, degraded)
- `/seano/communication/failure_alert` (Bool): Alert ketika SEMUA link down

**WiFi:**
- `/seano/communication/wifi/rssi` (Float32): WiFi RSSI (dBm)
- `/seano/communication/wifi/quality` (Float32): WiFi quality (%)
- `/seano/communication/wifi/status` (String): WiFi status (good, weak, critical, disconnected)

**GSM:**
- `/seano/communication/gsm/signal` (Float32): GSM signal strength (CSQ value)
- `/seano/communication/gsm/quality` (Float32): GSM quality (%)
- `/seano/communication/gsm/status` (String): GSM status (good, weak, critical, disconnected)

**Ethernet:**
- `/seano/communication/ethernet/status` (String): Ethernet status (connected, disconnected)
- `/seano/communication/ethernet/link` (Bool): Ethernet link up/down

**Parameters:**
- `failsafe.communication.check_interval` (float, default: 2.0): Interval check
- `failsafe.communication.wifi_interface` (string, default: wlP1p1s0): WiFi interface
- `failsafe.communication.gsm_interface` (string, default: wwan0): GSM interface  
- `failsafe.communication.ethernet_interface` (string, default: enP8p1s0): Ethernet interface
- `failsafe.communication.wifi_rssi_warning` (float, default: -70.0): Warning RSSI (dBm)
- `failsafe.communication.wifi_rssi_critical` (float, default: -80.0): Critical RSSI (dBm)
- `failsafe.communication.gsm_signal_warning` (float, default: 15.0): Warning signal (CSQ)
- `failsafe.communication.gsm_signal_critical` (float, default: 10.0): Critical signal (CSQ)
- `failsafe.communication.consecutive_failures` (int, default: 3): Jumlah failures sebelum alert

### 3. seano_failsafe
Node utama untuk handle failsafe procedures.

**Workflow:**
1. Deteksi kondisi critical (battery ATAU communication)
2. Kirim notifikasi via MQTT ke web dashboard
3. Tunggu notification_delay (2 detik)
4. Change mode Mavros ke RTL/Loiter/Land
5. Publish emergency stop signal

**Subscribed Topics:**
- `/seano/battery/low_alert` (Bool): Battery alert
- `/seano/battery/voltage` (Float32): Voltage saat ini
- `/seano/communication/failure_alert` (Bool): Communication alert
- `/seano/communication/rssi` (Float32): RSSI saat ini
- `/mavros/state` (State): Mavros state

**Published Topics:**
- `/seano/failsafe/status` (String): Status failsafe (INACTIVE, PENDING, ACTIVE)
- `/seano/failsafe/emergency_stop` (Bool): Emergency stop signal
- `/seano/failsafe/event` (String): Failsafe events (JSON)
- `/seano/mqtt/failsafe_notification` (String): MQTT notifications (JSON)

**Parameters:**
- `failsafe.system.battery_failsafe_enabled` (bool, default: true): Enable battery failsafe
- `failsafe.system.communication_failsafe_enabled` (bool, default: true): Enable comm failsafe
- `failsafe.system.failsafe_mode` (string, default: RTL): Mode untuk failsafe (RTL, LOITER, LAND)
- `failsafe.system.notification_delay` (float, default: 2.0): Delay sebelum action (detik)
- `failsafe.system.recovery_delay` (float, default: 10.0): Delay untuk recovery (detik)

## Dependencies

```bash
# Install pyserial untuk serial communication
pip3 install pyserial
```

## Build

```bash
cd /home/seano/Seano_ws
colcon build --packages-select seano_failsafe seano_startup
source install/setup.bash
```

## Usage

### Terintegrasi dengan seano_startup

```bash
ros2 launch seano_startup system.launch.py
```

### Konfigurasi

Edit [system.yaml](../seano_startup/config/system.yaml):

```yaml
failsafe:
  battery:
    serial_port: /dev/ttyTHS0      # Port serial ESP32
    baudrate: 115200
    min_voltage: 10.5               # 3S LiPo: 3.5V/cell
    max_voltage: 12.6               # 3S LiPo: 4.2V/cell
    low_voltage_threshold: 11.1     # Warning level
    critical_voltage_threshold: 10.8 # Critical level
  
  communication:
    check_interval: 2.0
    wifi_interface: wlP1p1s0        # WiFi interface name
    gsm_interface: wwan0             # GSM modem interface
    ethernet_interface: enP8p1s0    # Ethernet interface
    wifi_rssi_warning: -70.0         # WiFi weak signal (dBm)
    wifi_rssi_critical: -80.0        # WiFi critical signal (dBm)
    gsm_signal_warning: 15.0         # GSM weak signal (CSQ value)
    gsm_signal_critical: 10.0        # GSM critical signal (CSQ value)
    consecutive_failures: 3          # Failures before alert
  
  system:
    battery_failsafe_enabled: true
    communication_failsafe_enabled: true
    failsafe_mode: RTL              # RTL, LOITER, or LAND
    notification_delay: 2.0         # Delay before mode change
```

## Monitor Topics

```bash
# Battery
ros2 topic echo /seano/battery/voltage
ros2 topic echo /seano/battery/current
ros2 topic echo /seano/battery/percentage

# Communication - Aggregate
ros2 topic echo /seano/communication/status
ros2 topic echo /seano/communication/failure_alert

# Communication - WiFi
ros2 topic echo /seano/communication/wifi/rssi
ros2 topic echo /seano/communication/wifi/quality
ros2 topic echo /seano/communication/wifi/status

# Communication - GSM
ros2 topic echo /seano/communication/gsm/signal
ros2 topic echo /seano/communication/gsm/quality
ros2 topic echo /seano/communication/gsm/status

# Communication - Ethernet
ros2 topic echo /seano/communication/ethernet/status
ros2 topic echo /seano/communication/ethernet/link

# Failsafe
ros2 topic echo /seano/failsafe/status
ros2 topic echo /seano/failsafe/event
```

## ESP32 Serial Format

ESP32 harus kirim data dalam format:

**JSON Format (Recommended):**
```json
{"voltage": 12.5, "current": 2.3}
```

**Simple Format:**
```
V:12.5,A:2.3
```

Kirim data setiap 500ms - 1 detik via Serial ke Jetson.

## Features

- ✅ Real-time battery monitoring dari ESP32 via serial
- ✅ Multi-interface communication monitoring (WiFi + GSM + Ethernet)
- ✅ WiFi RSSI monitoring via iwconfig
- ✅ GSM signal monitoring via mmcli (ModemManager)
- ✅ Ethernet link status monitoring
- ✅ Failsafe trigger hanya jika SEMUA komunikasi down
- ✅ Dual trigger failsafe (battery + communication)
- ✅ MQTT notification sebelum mode change
- ✅ Automatic Mavros mode change (RTL/Loiter/Land)
- ✅ Emergency stop signal
- ✅ Configurable thresholds dan delays
- ✅ Recovery handling
- ✅ Terintegrasi dengan seano_startup
- ✅ Tidak ganggu seano_communication package

## Failsafe Flow

```
[Battery/Comm Critical Detected]
           ↓
[Send MQTT Notification (warning)]
           ↓
[Wait notification_delay (2s)]
           ↓
[Send MQTT Notification (critical)]
           ↓
[Change Mavros Mode to RTL]
           ↓
[Publish Emergency Stop Signal]
           ↓
[Monitor Recovery]
           ↓
[If recovered for recovery_delay (10s)]
           ↓
[Send MQTT Notification (recovery)]
           ↓
[Deactivate Failsafe]
```

## System Requirements

- Jetson Nano/Xavier dengan UART port
- ESP32 terhubung ke serial port
- Linux dengan:
  - iwconfig (wireless-tools) untuk WiFi monitoring
  - mmcli (ModemManager) untuk GSM monitoring
  - sysfs untuk Ethernet monitoring
- ROS2 Humble
- Mavros
- Python 3.8+
- pyserial

### Install Dependencies

```bash
# Install wireless tools for WiFi monitoring
sudo apt-get install wireless-tools

# Install ModemManager for GSM monitoring  
sudo apt-get install modemmanager

# Install pyserial
pip3 install pyserial
```

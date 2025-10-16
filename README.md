# Cognitive-Opportunistic Receiver System

A small end-to-end stack for reading sensor data from an Arduino-based payload (GPS, barometer, IMU), forwarding it over UDP/serial, and visualizing it on a desktop UI (PyQt5). The UI is designed to run well on a Raspberry Pi display (“LEO-PNT Pi UI”) but works on any desktop.

## Repository Layout

- `Arduino/main.c` — Arduino sketch that outputs CSV and JSON lines with readings from:
  - GPS (`TinyGPSPlus` via `SoftwareSerial`)
  - Barometer/temperature `BMP180` (`Adafruit_BMP085`)
  - IMU `MPU6050` (DMP)
- `backend/backend.py` — Python bridge that reads serial CSV/JSON and forwards unified JSON packets via UDP.
- `backend/debug_serial.py` — Simple serial line dumper for verifying data is coming from the board.
- `frontend/leo_pnt_ui.py` — PyQt5 dashboard (tabs for GPS, Barometer, Temperature, SDR sim, IMU) and a “Dashboard” summary view. Supports:
  - UDP ingestion (binds a local UDP port) for JSON packets
  - Optional serial ingestion (auto-detect or env var)

## Data Flow

1) Arduino prints data as CSV lines with a header or as JSON objects (one per line). Example JSON messages the UI/back-end understand:

```json
{"sensor":"gps","lat":12.345678,"lon":98.765432,"alt":250.1,"fix":"3D","sats":8,"hdop":0.9,"time":"UTC"}
{"sensor":"baro","pressure_hpa":1013.7,"alt_m":35.4,"temp_c":26.2}
{"sensor":"temp","temp_c":26.2,"status":"OK"}
{"sensor":"imu","yaw_deg":12.0,"pitch_deg":-3.1,"roll_deg":0.6}
```

2) `backend/backend.py` can convert CSV to the same JSON and send them via UDP.

3) `frontend/leo_pnt_ui.py` listens on UDP and/or reads serial directly, then updates the UI in near‑real time.

## Quick Start (UI)

Requirements:
- Python 3.9+
- `pip install PyQt5 pyserial`

Run the UI:
- `python frontend/leo_pnt_ui.py`

The UI opens tabs for each sensor and a consolidated dashboard. It also exposes a UDP listener and, if available, tries to auto-connect to a serial port.

## Quick Start (Backend bridge)

`backend/backend.py` reads your Arduino’s serial output and forwards JSON by UDP.

Update these constants as needed (Windows example uses `COM3`; Linux uses `/dev/ttyUSB0`):

```python
PORT = "COM3"      # or "/dev/ttyUSB0"
BAUD = 115200
UDP_ADDR = ("127.0.0.1", 9000)  # match the UI UDP port
```

Run:
- `python backend/backend.py`

Note: The current UI binds UDP on port `9000`. Ensure `UDP_ADDR` in `backend.py` matches this (the sample code originally targeted `9999`).

## Serial Auto-Connect (UI)

The UI attempts to auto-connect to a serial port. To force a port, set an environment variable before launching the UI:
- On Windows (PowerShell):
  - `$env:PI_SERIAL_PORT = "COM3"; python frontend/leo_pnt_ui.py`
- On Linux/macOS (bash):
  - `PI_SERIAL_PORT=/dev/ttyUSB0 python3 frontend/leo_pnt_ui.py`

## CSV Format (Arduino)

The sketch prints a header then CSV rows. Expected header columns (example):
`time_ms,lat,lon,gps_alt_m,fix,sats,hdop,bmp_temp_c,bmp_pres_hpa,bmp_alt_m,yaw_deg,pitch_deg,roll_deg`

The backend and UI parse these into JSON packets for each sensor.

## Testing via UDP (no hardware)

You can inject JSON packets directly to the UI’s UDP port for testing. Example (Python):

```python
import socket, json
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
def send(obj):
    sock.sendto((json.dumps(obj)+"\n").encode("utf-8"), ("127.0.0.1", 9000))
send({"sensor":"gps","lat":37.7749,"lon":-122.4194,"alt":20.0,"fix":"3D","sats":7,"hdop":0.8,"time":"UTC"})
send({"sensor":"baro","pressure_hpa":1012.8,"alt_m":22.3,"temp_c":24.1})
send({"sensor":"imu","yaw_deg":10.0,"pitch_deg":-1.5,"roll_deg":0.4})
```

## Arduino Setup

Libraries:
- `TinyGPSPlus`
- `Adafruit_BMP085` (BMP180)
- `I2Cdev` and `MPU6050_6Axis_MotionApps20`

Upload `Arduino/main.c` to your board (e.g., Arduino Uno). The sketch outputs both CSV and select JSON lines at 115200 baud.

## Notes & Troubleshooting

- UDP Port: UI listens on `9000`. Set `backend/backend.py` `UDP_ADDR` to `(127.0.0.1, 9000)`.
- Serial Ports: On Windows use `COMx` (e.g., `COM3`). On Linux/macOS use `/dev/ttyUSB0` or `/dev/ttyACM0`.
- Merge Markers: If you see `<<<<<<<`/`=======`/`>>>>>>>` markers in `frontend/leo_pnt_ui.py`, resolve them before production use.
- Temperature units: UI displays Celsius.

## License

No license file is provided. Add one if you plan to share/distribute.



import json, sys, time
import serial
import socket

PORT = "/dev/ttyUSB0"   
BAUD = 115200
UDP_ADDR = ("127.0.0.1", 9999)
ser = serial.Serial(PORT, BAUD, timeout=0.5)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print(f"[csv->json] {PORT} @ {BAUD} -> UDP {UDP_ADDR}")
header = None

def send(obj):
    data = (json.dumps(obj) + "\n").encode("utf-8")
    sock.sendto(data, UDP_ADDR)

while True:
    try:
        line = ser.readline()
        if not line:
            continue
        s = line.decode(errors="ignore").strip()
        if not s:
            continue

        if s.startswith("time_ms,"):
            header = s.split(",")
            continue
        if s.startswith("GMAPS:") or s.startswith("{"):
            
            try:
                obj = json.loads(s)
                if isinstance(obj, dict) and "sensor" in obj:
                    send(obj)
            except:
                pass
            continue
        if header:
            parts = s.split(",")
            if len(parts) != len(header):
                continue

            row = dict(zip(header, parts))

            
            def f(name):
                v = row.get(name, "NA")
                try:
                    return float(v) if v != "NA" else None
                except:
                    return None

            
            lat, lon = f("lat"), f("lon")
            alt = f("gps_alt_m")
            fix = "3D" if row.get("fix","0") not in ("0","NA") else "No Fix"
            sats = int(float(row.get("sats","0"))) if row.get("sats","0").replace('.','',1).isdigit() else 0
            hdop = f("hdop")

            if lat is not None and lon is not None:
                send({"sensor":"gps", "lat":lat, "lon":lon, "alt":alt or 0.0,
                      "fix":fix, "sats":sats, "hdop":hdop or 0.0, "time":"UTC"})

            send({"sensor":"baro",
                  "pressure_hpa": f("bmp_pres_hpa") or 0.0,
                  "alt_m": f("bmp_alt_m") or 0.0,
                  "temp_c": f("bmp_temp_c") or 0.0})

            send({"sensor":"temp",
                  "temp_c": f("bmp_temp_c") or 0.0,
                  "humidity": 0,
                  "status":"OK"})
            send({
            "sensor": "imu",
            "yaw_deg":   f("yaw_deg")   or 0.0,
            "pitch_deg": f("pitch_deg") or 0.0,
            "roll_deg":  f("roll_deg")  or 0.0
            })
            yaw  = f("yaw_deg")
            pitch= f("pitch_deg")
            roll = f("roll_deg")
            if (yaw is not None) or (pitch is not None) or (roll is not None):
                send({"sensor":"imu",
                    "yaw_deg":  yaw  if yaw  is not None else 0.0,
                    "pitch_deg":pitch if pitch is not None else 0.0,
                    "roll_deg": roll if roll is not None else 0.0})
    except KeyboardInterrupt:
        break
    except Exception as e:
        print("[err]", e)
        time.sleep(0.5)

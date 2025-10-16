
import serial
import sys

PORT = "/dev/ttyUSB0" 
BAUD = 115200

try:
    ser = serial.Serial(PORT, BAUD, timeout=1.0)
    print(f"Connected to {PORT} @ {BAUD}")
    print("=" * 60)

    line_count = 0
    while line_count < 50: 
        try:
            line = ser.readline()
            if line:
                text = line.decode(errors='ignore').strip()
                if text:
                    line_count += 1
                    print(f"[{line_count:03d}] {text}")
        except KeyboardInterrupt:
            break

    ser.close()
    print("=" * 60)
    print("Done!")

except Exception as e:
    print(f"Error: {e}")
    print("Available ports:")
    import serial.tools.list_ports as list_ports
    for p in list_ports.comports():
        print(f"  - {p.device}")



import sys, os, time, math, random, json
from datetime import datetime, timezone
from PyQt5 import QtCore, QtGui, QtWidgets, QtNetwork


try:
    import serial
    import serial.tools.list_ports as serial_ports
except Exception:
    serial = None
    serial_ports = None


def nice(v, nd=3):
    try:
        return f"{float(v):.{nd}f}"
    except Exception:
        return str(v)

def now_iso():
    return datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S UTC")


class SensorWidget(QtWidgets.QWidget):
    updated = QtCore.pyqtSignal(dict)
    def __init__(self, title, field_specs, parent=None):
        super().__init__(parent)
        self.title = title
        self.field_specs = field_specs
        self.fields = {}
        self._build()

    def _build(self):
        v = QtWidgets.QVBoxLayout(self)
        hdr = QtWidgets.QHBoxLayout()
        t = QtWidgets.QLabel(f"<b>{self.title}</b>")
        self.status_dot = QtWidgets.QLabel(""); self.status_dot.setStyleSheet("color:#888; font-size:18px;")
        self.status_txt = QtWidgets.QLabel("Idle")
        hdr.addWidget(t, 1); hdr.addStretch(1); hdr.addWidget(self.status_dot); hdr.addWidget(self.status_txt)
        v.addLayout(hdr)

        form = QtWidgets.QFormLayout()
        for label, unit in self.field_specs:
            ln = QtWidgets.QLineEdit(); ln.setReadOnly(True); ln.setAlignment(QtCore.Qt.AlignRight)
            roww = QtWidgets.QWidget(); row = QtWidgets.QHBoxLayout(roww); row.setContentsMargins(0,0,0,0)
            row.addWidget(ln)
            if unit:
                u = QtWidgets.QLabel(unit); u.setStyleSheet("color:#666; margin-left:6px;")
                row.addWidget(u)
            row.addStretch(1)
            form.addRow(label + ":", roww)
            self.fields[label] = ln
        v.addLayout(form)
        self.last_update = QtWidgets.QLabel("Last update:"); self.last_update.setStyleSheet("color:#666;")
        v.addWidget(self.last_update); v.addStretch(1)

    def set_status(self, txt, color):
        self.status_txt.setText(txt)
        self.status_dot.setStyleSheet(f"color:{color}; font-size:18px;")

    def update_fields(self, data):
        for label, w in self.fields.items():
            if label in data:
                w.setText(nice(data[label], 4))
        self.set_status("Streaming", "#2ecc71")
        self.last_update.setText(f"Last update: {now_iso()}")
        self.updated.emit(data)


class Dashboard(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._build()
    def _build(self):
        grid = QtWidgets.QGridLayout(self)
        self.lat = self._card("Latitude","deg"); self.lon = self._card("Longitude","deg"); self.alt = self._card("Altitude","m")
        self.hdop = self._card("HDOP",""); self.sats = self._card("Sats Used",""); self.fix = self._card("Fix Type","")
        self.time_lbl = QtWidgets.QLabel(""); self.time_lbl.setAlignment(QtCore.Qt.AlignCenter)
        grid.addWidget(self._group("Position (WGS-84)", [self.lat,self.lon,self.alt]), 0,0)
        grid.addWidget(self._group("Quality/Time", [self.hdop,self.sats,self.fix], self.time_lbl), 0,1)
        foot = QtWidgets.QHBoxLayout(); self.runtime = QtWidgets.QLabel("Uptime: 00:00:00"); foot.addWidget(self.runtime); foot.addStretch(1)
        grid.addLayout(foot, 1,0,1,2)
    def _card(self, label, unit):
        w=QtWidgets.QWidget(); l=QtWidgets.QVBoxLayout(w)
        lab = QtWidgets.QLabel(label); lab.setStyleSheet("font-size:14px;")
        l.addWidget(lab)
        val=QtWidgets.QLabel(""); val.setStyleSheet("font:600 18px;")
        l.addWidget(val)
        if unit: l.addWidget(QtWidgets.QLabel(unit))
        return w
    def _group(self, title, cards, extra=None):
        g=QtWidgets.QGroupBox(title); g.setStyleSheet("QGroupBox{font-weight:600;}")
        lay=QtWidgets.QGridLayout(g)
        for i,c in enumerate(cards): lay.addWidget(c, i//2, i%2)
        if extra: lay.addWidget(extra, (len(cards)+1)//2, 0, 1, 2)
        return g
    def set_value(self, card, txt): card.layout().itemAt(1).widget().setText(txt)
    def update_solution(self, sol):
        if "lat" in sol and sol["lat"] is not None:
            self.set_value(self.lat, nice(sol["lat"], 6))
        if "lon" in sol and sol["lon"] is not None:
            self.set_value(self.lon, nice(sol["lon"], 6))
        if "alt" in sol and sol["alt"] is not None:
            self.set_value(self.alt, nice(sol["alt"], 2))
        if "hdop" in sol and sol["hdop"] is not None:
            self.set_value(self.hdop, nice(sol["hdop"], 2))
        if "sats" in sol and sol["sats"] is not None:
            self.set_value(self.sats, str(sol["sats"]))
        if "fix" in sol and sol["fix"] is not None:
            self.set_value(self.fix, str(sol["fix"]))
        if "time" in sol:
            self.time_lbl.setText(sol.get("time", now_iso()))
    def set_runtime(self, sec):
        h=sec//3600; m=(sec%3600)//60; s=sec%60; self.runtime.setText(f"Uptime: {h:02d}:{m:02d}:{s:02d}")

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("LEO-PNT Pi UI")
        self.resize(800, 480)
        self.showMaximized()

        self.latest = {"gps": {}, "baro": {}, "temp": {}, "sdr": {}, "imu": {}}

        self.serial_thread = None
        self.udp_listener = None
        self.system_running = False

        self._build_ui()
        self._build_timer()
        self._auto_connect_serial()

    def _build_ui(self):
        central=QtWidgets.QWidget(); self.setCentralWidget(central)
        v=QtWidgets.QVBoxLayout(central)

        ctl=QtWidgets.QHBoxLayout()
        self.btn_start = QtWidgets.QPushButton("Start System")
        self.btn_start.setStyleSheet("font-weight:bold; font-size:14px; padding:8px;")
        self.btn_stop = QtWidgets.QPushButton("Stop System")
        self.btn_stop.setEnabled(False)
        self.btn_quit = QtWidgets.QPushButton("Quit")
        ctl.addWidget(self.btn_start); ctl.addWidget(self.btn_stop); ctl.addStretch(1); ctl.addWidget(self.btn_quit)
        v.addLayout(ctl)

        self.tabs = QtWidgets.QTabWidget(); v.addWidget(self.tabs, 1)
        self.dashboard = Dashboard(); self.tabs.addTab(self.dashboard, "Dashboard")

<<<<<<< HEAD
        baro_fields=[("Pressure","hPa"),("Altitude (baro)","m"),("Temperature","C")]
        temp_fields=[("Temperature","C"),("Humidity","%"),("Sensor Status","")]
=======
        baro_fields=[("Pressure","hPa"),("Altitude (baro)","m"),("Temperature","°C")]
        temp_fields=[("Temperature","°C"),("Sensor Status","")]
>>>>>>> b234c3d421457bc9bdda67edfa4d5c3a9976e961
        gps_fields =[("Latitude","deg"),("Longitude","deg"),("Altitude","m"),("Fix Type",""),("Sats Used",""),("HDOP",""),("UTC Time","")]
        sdr_fields =[("Frequency","Hz"),("Doppler","Hz"),("SNR","dB-Hz"),("RSSI","dBm"),("Bandwidth","kHz"),("Constellation","")]
        imu_fields =[("Yaw","deg"),("Pitch","deg"),("Roll","deg")]

        self.sensor_baro=SensorWidget("Barometer (BMP180)", baro_fields)
        self.sensor_temp=SensorWidget("Temperature (BMP180)", temp_fields)
        self.sensor_gps =SensorWidget("GPS (NEO-6M)", gps_fields)
        self.sensor_sdr =SensorWidget("SDR Radio (Sim when no RTL)", sdr_fields)
        self.sensor_imu =SensorWidget("IMU (MPU6050 DMP)", imu_fields)

        self.tabs.addTab(self.sensor_baro,"Barometer"); self.tabs.addTab(self.sensor_temp,"Temperature")
        self.tabs.addTab(self.sensor_gps,"GPS"); self.tabs.addTab(self.sensor_sdr,"SDR Radio"); self.tabs.addTab(self.sensor_imu,"IMU")

        self.btn_start.clicked.connect(self.start_system)
        self.btn_stop.clicked.connect(self.stop_system)
        self.btn_quit.clicked.connect(self.close)

    def _build_timer(self):
        self.timer=QtCore.QTimer(self); self.timer.setInterval(200); self.timer.timeout.connect(self._tick)

    def start_system(self):
        if self.system_running:
            return

        self.system_running = True
        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)

        self.start_udp()

        self.timer.start()

    def stop_system(self):
        if not self.system_running:
            return

        self.system_running = False
        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)

        self.stop_udp()

        self.timer.stop()

<<<<<<< HEAD
=======
    # ---- UDP Listener (Port 9999 - matches backend.py) ----
>>>>>>> b234c3d421457bc9bdda67edfa4d5c3a9976e961
    def start_udp(self):
        if self.udp_listener is not None:
            return

        self.udp_listener = QtNetwork.QUdpSocket(self)
        if not self.udp_listener.bind(9999):
            QtWidgets.QMessageBox.warning(self, "UDP Error", "Failed to bind to port 9999. Port may be in use.")
            self.udp_listener = None
            return

        self.udp_listener.readyRead.connect(self._on_udp_datagram)
        print("[UDP] Listening on port 9999")

    def stop_udp(self):
        if self.udp_listener is not None:
            self.udp_listener.close()
            self.udp_listener = None

    def _on_udp_datagram(self):
        while self.udp_listener and self.udp_listener.hasPendingDatagrams():
            datagram_size = self.udp_listener.pendingDatagramSize()
            datagram, host, port = self.udp_listener.readDatagram(datagram_size)

            try:
                text = datagram.decode('utf-8').strip()
                if text.startswith("{") and text.endswith("}"):
                    obj = json.loads(text)
                    if isinstance(obj, dict):
                        self._on_external_packet(obj)
            except Exception:
                pass

    def _tick(self):
        t = time.time()
        if not hasattr(self, "_start_ts"):
            self._start_ts = t

        freq = 1_575_420_000.0
        doppler = 18000.0 * math.sin(2*math.pi*((t - self._start_ts)/17.5)) + random.gauss(0, 30)
        snr = max(23, min(55, 40 + 8*math.sin((t - self._start_ts)/6.0) + random.gauss(0, 1.0)))
        rssi = -70.0 + 2.5*math.sin((t - self._start_ts)/8.0) + random.gauss(0, 0.4)
        bw = 250.0
        constel = "SoOP (sim)"
        self.sensor_sdr.update_fields({"Frequency":freq,"Doppler":doppler,"SNR":snr,"RSSI":rssi,"Bandwidth":bw,"Constellation":constel})
        self.latest["sdr"]={"freq":freq,"doppler":doppler,"snr":snr,"rssi":rssi,"bw_khz":bw,"constellation":constel}

        if self.latest["gps"]:
            self.dashboard.update_solution({
                "lat": self.latest["gps"].get("lat"),
                "lon": self.latest["gps"].get("lon"),
                "alt": self.latest["gps"].get("alt"),
                "hdop": self.latest["gps"].get("hdop"),
                "sats": self.latest["gps"].get("sats"),
                "fix" : self.latest["gps"].get("fix"),
                "time": self.latest["gps"].get("time", now_iso())
            })

        self.dashboard.set_runtime(int(t - self._start_ts))

    def ingest_barometer(self, pressure_hpa, alt_m, temp_c):
        self.sensor_baro.update_fields({"Pressure":pressure_hpa,"Altitude (baro)":alt_m,"Temperature":temp_c})
        self.latest["baro"]={"pressure":pressure_hpa,"alt_m":alt_m,"temp_c":temp_c}

    def ingest_temperature(self, temp_c, status="OK"):
        self.sensor_temp.update_fields({"Temperature":temp_c,"Sensor Status":status})
        self.latest["temp"]={"temp_c":temp_c,"status":status}

    def ingest_gps(self, lat, lon, alt_m, fix, sats, hdop, utc_iso=None):
        self.sensor_gps.update_fields({
            "Latitude":lat,"Longitude":lon,"Altitude":alt_m,"Fix Type":fix,
            "Sats Used":sats,"HDOP":hdop,"UTC Time": utc_iso or now_iso()
        })
        self.latest["gps"]={"lat":lat,"lon":lon,"alt":alt_m,"fix":fix,"sats":sats,"hdop":hdop,"time":utc_iso or now_iso()}
        self.dashboard.update_solution({"lat":lat,"lon":lon,"alt":alt_m,"fix":fix,"sats":sats,"hdop":hdop,"time": utc_iso or now_iso()})

    def ingest_imu(self, yaw, pitch, roll):
        self.sensor_imu.update_fields({"Yaw":yaw, "Pitch":pitch, "Roll":roll})
        self.latest["imu"]={"yaw":yaw,"pitch":pitch,"roll":roll}

    def _auto_connect_serial(self):
        if not serial:
            QtWidgets.QMessageBox.warning(self, "Serial", "pyserial is not installed.")
            return

        prefer = os.environ.get("PI_SERIAL_PORT", "").strip()
        port = None
        if prefer:
            port = prefer
        else:
            ports = [p.device for p in serial_ports.comports()] if serial_ports else []
            for cand in ports:
                if ("/dev/ttyUSB" in cand) or ("/dev/ttyACM" in cand) or ("COM" in cand):
                    port = cand; break

        if not port:
            QtWidgets.QMessageBox.warning(self, "Serial", "No serial port found (USB/ACM). Plug Arduino and reboot this app.")
            return

        baud = int(os.environ.get("PI_SERIAL_BAUD","115200"))
        self.serial_thread = SerialReaderFlexible(port, baud)
        self.serial_thread.got_packet.connect(self._on_external_packet)
        self.serial_thread.start()

    def _on_external_packet(self, obj):
        s = obj.get("sensor","").lower()
        print(f"[PKT] sensor={s}, data={obj}")  # DEBUG

        if s=="gps":
            lat = float(obj.get("lat",0))
            lon = float(obj.get("lon",0))
            alt = float(obj.get("alt",0))
            fix = str(obj.get("fix",""))
            sats = int(obj.get("sats",0))
            hdop = float(obj.get("hdop",0.0))
            print(f"[GPS] lat={lat}, lon={lon}, alt={alt}, sats={sats}, hdop={hdop}")  # DEBUG
            self.ingest_gps(lat, lon, alt, fix, sats, hdop, obj.get("time") or now_iso())

        elif s=="baro":
            pressure = float(obj.get("pressure_hpa",1013.25))
            alt_m = float(obj.get("alt_m",0.0))
            temp = float(obj.get("temp_c",0.0))
            print(f"[BARO] pressure={pressure}, alt={alt_m}, temp={temp}")  # DEBUG
            self.ingest_barometer(pressure, alt_m, temp)

        elif s=="temp":
            temp = float(obj.get("temp_c",0.0))
            print(f"[TEMP] temp={temp}")  # DEBUG
            self.ingest_temperature(temp, str(obj.get("status","OK")))

        elif s=="imu":
            yaw = float(obj.get("yaw_deg",0.0))
            pitch = float(obj.get("pitch_deg",0.0))
            roll = float(obj.get("roll_deg",0.0))
            print(f"[IMU] yaw={yaw}, pitch={pitch}, roll={roll}")  # DEBUG
            self.ingest_imu(yaw, pitch, roll)

    def closeEvent(self, e: QtGui.QCloseEvent):
        try:
            if self.system_running:
                self.stop_system()
            if self.serial_thread:
                self.serial_thread.stop(); self.serial_thread.wait(1500)
        except: pass
        super().closeEvent(e)

class SerialReaderFlexible(QtCore.QThread):
    got_packet = QtCore.pyqtSignal(dict)
    def __init__(self, port, baud=115200, parent=None):
        super().__init__(parent); self.port=port; self.baud=baud; self._stop=False; self.ser=None
        self.header = None
    def run(self):
        if not serial: return
        try: self.ser=serial.Serial(self.port,self.baud,timeout=0.5)
        except Exception: return
        while not self._stop:
            try:
                line=self.ser.readline()
                if not line: continue
                s=line.decode(errors="ignore").strip()
                if not s: continue

                if s.startswith("{") and s.endswith("}"):
                    try:
                        obj = json.loads(s)
                        if isinstance(obj, dict): self.got_packet.emit(obj)
                        continue
                    except Exception:
                        pass

                if s.startswith("time_ms,"):
                    self.header = s.split(","); continue

                if self.header and ("," in s) and (not s.startswith("GMAPS:")):
                    parts = s.split(",")
                    if len(parts) == len(self.header):
                        row = dict(zip(self.header, parts))
                        def f(name):
                            v = row.get(name, "NA")
                            try:
                                return float(v) if v != "NA" else None
                            except:
                                return None

<<<<<<< HEAD
=======
                        # GPS - always send if we have valid lat/lon
>>>>>>> b234c3d421457bc9bdda67edfa4d5c3a9976e961
                        lat, lon = f("lat"), f("lon")
                        alt = f("gps_alt_m")
                        fix = "3D" if row.get("fix","0") not in ("0","NA") else "No Fix"
                        sats = int(float(row.get("sats","0"))) if str(row.get("sats","0")).replace('.','',1).isdigit() else 0
                        hdop = f("hdop")
                        if lat is not None and lon is not None:
<<<<<<< HEAD
                            self.got_packet.emit({"sensor":"gps","lat":lat,"lon":lon,"alt":alt or 0.0,
                                                  "fix":fix,"sats":sats,"hdop":hdop or 0.0,"time":"UTC"})
                        self.got_packet.emit({"sensor":"baro",
                                              "pressure_hpa": f("bmp_pres_hpa") or 0.0,
                                              "alt_m": f("bmp_alt_m") or 0.0,
                                              "temp_c": f("bmp_temp_c") or 0.0})
                        self.got_packet.emit({"sensor":"temp",
                                              "temp_c": f("bmp_temp_c") or 0.0,
                                              "humidity": 0,
                                              "status":"OK"})
=======
                            self.got_packet.emit({"sensor":"gps","lat":lat,"lon":lon,"alt":alt if alt is not None else 0.0,
                                                  "fix":fix,"sats":sats,"hdop":hdop if hdop is not None else 0.0,"time":"UTC"})

                        # Barometer - always send pressure data
                        bmp_pres = f("bmp_pres_hpa")
                        bmp_alt = f("bmp_alt_m")
                        bmp_temp = f("bmp_temp_c")
                        if bmp_pres is not None or bmp_alt is not None or bmp_temp is not None:
                            self.got_packet.emit({"sensor":"baro",
                                                  "pressure_hpa": bmp_pres if bmp_pres is not None else 1013.25,
                                                  "alt_m": bmp_alt if bmp_alt is not None else 0.0,
                                                  "temp_c": bmp_temp if bmp_temp is not None else 0.0})

                        # Temperature sensor (separate from baro temp)
                        if bmp_temp is not None:
                            self.got_packet.emit({"sensor":"temp",
                                                  "temp_c": bmp_temp,
                                                  "status":"OK"})

                        # IMU (Yaw/Pitch/Roll) - always send if any value is present
>>>>>>> b234c3d421457bc9bdda67edfa4d5c3a9976e961
                        yd, pd, rd = f("yaw_deg"), f("pitch_deg"), f("roll_deg")
                        if (yd is not None) or (pd is not None) or (rd is not None):
                            self.got_packet.emit({"sensor":"imu",
                                                  "yaw_deg": yd if yd is not None else 0.0,
                                                  "pitch_deg": pd if pd is not None else 0.0,
                                                  "roll_deg": rd if rd is not None else 0.0})
                    continue

            except Exception:
                break
        try:
            if self.ser: self.ser.close()
        except: pass
    def stop(self): self._stop=True

def main():
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    pal = app.palette(); pal.setColor(QtGui.QPalette.Window, QtGui.QColor("#f7f7fb"))
    pal.setColor(QtGui.QPalette.Base, QtGui.QColor("#ffffff")); app.setPalette(pal)
    w=MainWindow(); w.show()
    sys.exit(app.exec_())

if __name__=="__main__":
    main()

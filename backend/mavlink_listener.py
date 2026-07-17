"""
XO11 GCS — MAVLink Listener (Module 5)

Receives MAVLink telemetry over UDP (default udpin:0.0.0.0:14550) and
converts it into the same telemetry dict shape the internal simulator
produces, so the dashboard is source-agnostic.

Works with:
  - mavlink_sim_sender.py (bundled simulated MAVLink stream)
  - SITL (ArduPilot / PX4)  e.g.  sim_vehicle.py --out=udp:127.0.0.1:14550
  - A real UAV telemetry radio

Requires: pip install pymavlink
"""

import threading
import time

try:
    from pymavlink import mavutil
    HAVE_PYMAVLINK = True
except ImportError:
    HAVE_PYMAVLINK = False

# ArduPilot copter custom-mode -> friendly name (subset)
COPTER_MODES = {
    0: "MANUAL",     # Stabilize
    3: "AUTO",
    4: "AUTO",       # Guided
    5: "HOLD",       # Loiter
    6: "RTH",        # RTL
    9: "LAND",
    16: "HOLD",      # PosHold
}


class MAVLinkListener:
    """Background thread that ingests MAVLink and keeps latest telemetry."""

    def __init__(self, conn_str="udpin:0.0.0.0:14550"):
        if not HAVE_PYMAVLINK:
            raise RuntimeError("pymavlink is not installed — pip install pymavlink")
        self.conn_str = conn_str
        self._lock = threading.Lock()
        self._state = {}
        self._events = []
        self._last_heartbeat = 0.0
        self._last_mode = None
        self._running = False
        self._thread = None

    # ------------------------------------------------------------------
    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False

    @property
    def connected(self):
        return (time.time() - self._last_heartbeat) < 5.0

    def drain_events(self):
        with self._lock:
            ev, self._events = self._events, []
            return ev

    def snapshot(self):
        with self._lock:
            if not self._state:
                return None
            s = dict(self._state)
            s["ts"] = time.time()
            s["link_ok"] = self.connected
            return s

    # ------------------------------------------------------------------
    def _run(self):
        conn = mavutil.mavlink_connection(self.conn_str)
        while self._running:
            msg = conn.recv_match(blocking=True, timeout=1.0)
            if msg is None:
                continue
            t = msg.get_type()
            with self._lock:
                if t == "HEARTBEAT":
                    self._last_heartbeat = time.time()
                    mode = COPTER_MODES.get(getattr(msg, "custom_mode", -1), "MANUAL")
                    armed = bool(msg.base_mode & 128)  # MAV_MODE_FLAG_SAFETY_ARMED
                    if mode != self._last_mode:
                        if self._last_mode is not None:
                            self._events.append(
                                ("MODE", f"MAVLink mode change: {self._last_mode} -> {mode}"))
                        self._last_mode = mode
                    self._state["mode"] = mode
                    self._state["armed"] = armed

                elif t == "GLOBAL_POSITION_INT":
                    self._state["lat"] = msg.lat / 1e7
                    self._state["lon"] = msg.lon / 1e7
                    self._state["alt"] = msg.relative_alt / 1000.0
                    self._state["heading"] = msg.hdg / 100.0 if msg.hdg != 65535 else 0.0
                    # vx/vy in cm/s -> ground speed km/h
                    gs = (msg.vx ** 2 + msg.vy ** 2) ** 0.5 / 100.0
                    self._state["groundspeed"] = round(gs * 3.6, 1)
                    self._state["climb"] = round(-msg.vz / 100.0, 2)

                elif t == "VFR_HUD":
                    self._state["airspeed"] = round(msg.airspeed * 3.6, 1)   # m/s -> km/h
                    self._state["groundspeed"] = round(msg.groundspeed * 3.6, 1)
                    self._state["alt"] = round(msg.alt, 1)
                    self._state["climb"] = round(msg.climb, 2)

                elif t == "SYS_STATUS":
                    self._state["battery_voltage"] = round(msg.voltage_battery / 1000.0, 2)
                    self._state["current_a"] = round(msg.current_battery / 100.0, 1)
                    self._state["battery_pct"] = float(msg.battery_remaining)

                elif t == "GPS_RAW_INT":
                    self._state["sats"] = msg.satellites_visible
                    self._state["hdop"] = round(msg.eph / 100.0, 2) if msg.eph != 65535 else None

                elif t == "RADIO_STATUS":
                    # rssi 0..254 -> percent
                    self._state["signal_pct"] = round(msg.rssi / 254.0 * 100.0, 1)

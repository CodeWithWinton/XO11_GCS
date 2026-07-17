"""
XO11 UAV Systems — Ground Control Station server

Serves the dashboard, streams telemetry via Server-Sent Events, and
exposes a small REST API for mission planning and flight commands.

Telemetry sources (env GCS_SOURCE):
  sim      (default) — built-in calibrated flight simulator
  mavlink            — pymavlink listener on udpin:0.0.0.0:14550
                       (pair with mavlink_sim_sender.py or a real UAV)

Run:  python app.py           ->  http://localhost:5000
"""

import csv
import io
import json
import os
import queue
import threading
import time
from collections import deque
from datetime import datetime, timezone

from flask import Flask, Response, jsonify, request, send_from_directory

from simulator import UAVSimulator

FRONTEND_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "..", "frontend")
TICK_HZ = 4.0
SOURCE = os.environ.get("GCS_SOURCE", "sim").lower()

app = Flask(__name__, static_folder=None)

# ---------------------------------------------------------------------------
# Mission log (Module 4)
# ---------------------------------------------------------------------------
class MissionLog:
    def __init__(self, maxlen=5000):
        self._lock = threading.Lock()
        self._entries = deque(maxlen=maxlen)

    def add(self, etype, message, severity="info"):
        with self._lock:
            entry = {
                "ts": datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds"),
                "type": etype,
                "severity": severity,
                "message": message,
            }
            self._entries.append(entry)
            return entry

    def all(self):
        with self._lock:
            return list(self._entries)

    def as_txt(self):
        return "\n".join(f"[{e['ts']}] [{e['severity'].upper():8s}] "
                         f"[{e['type']:9s}] {e['message']}"
                         for e in self.all()) + "\n"

    def as_csv(self):
        buf = io.StringIO()
        w = csv.DictWriter(buf, fieldnames=["ts", "type", "severity", "message"])
        w.writeheader()
        w.writerows(self.all())
        return buf.getvalue()


# ---------------------------------------------------------------------------
# Alert engine (Module 3) — thresholds with hysteresis so alerts don't flap
# ---------------------------------------------------------------------------
class AlertEngine:
    """
    Each rule: (id, severity, trigger_fn, clear_fn, message_fn)
    Hysteresis: trigger and clear thresholds differ, so a value hovering
    at the boundary doesn't spam alerts.
    """

    def __init__(self, log):
        self.log = log
        self.active = {}   # id -> alert dict
        self._lock = threading.Lock()
        self.rules = [
            ("BATTERY_CRITICAL", "critical",
             lambda t: t.get("battery_pct", 100) < 15,
             lambda t: t.get("battery_pct", 100) >= 17,
             lambda t: f"Battery CRITICAL: {t['battery_pct']:.0f}% "
                       f"({t.get('battery_voltage', 0):.1f} V) — land immediately"),
            ("BATTERY_LOW", "warning",
             lambda t: t.get("battery_pct", 100) < 25,
             lambda t: t.get("battery_pct", 100) >= 27,
             lambda t: f"Battery LOW: {t['battery_pct']:.0f}% "
                       f"({t.get('battery_voltage', 0):.1f} V)"),
            ("GPS_LOST", "warning",
             lambda t: t.get("sats", 99) < 6,
             lambda t: t.get("sats", 99) >= 7,
             lambda t: f"GPS degraded: only {t['sats']} satellites locked"),
            ("SIGNAL_WEAK", "warning",
             lambda t: t.get("signal_pct", 100) < 35,
             lambda t: t.get("signal_pct", 100) >= 45,
             lambda t: f"Signal WEAK: link quality {t['signal_pct']:.0f}%"),
            ("LINK_LOST", "critical",
             lambda t: t.get("link_ok") is False,
             lambda t: t.get("link_ok", True) is True,
             lambda t: "Telemetry link LOST — no heartbeat"),
        ]

    def evaluate(self, tele):
        """Returns list of newly raised/cleared alert events."""
        changes = []
        with self._lock:
            # BATTERY_CRITICAL suppresses BATTERY_LOW duplication
            crit_active = "BATTERY_CRITICAL" in self.active
            for aid, sev, trig, clear, msg in self.rules:
                if aid == "BATTERY_LOW" and (crit_active or
                                             tele.get("battery_pct", 100) < 15):
                    continue
                if aid not in self.active:
                    try:
                        fire = trig(tele)
                    except (TypeError, KeyError):
                        fire = False
                    if fire:
                        alert = {"id": aid, "severity": sev,
                                 "message": msg(tele),
                                 "raised_at": datetime.now(timezone.utc)
                                 .astimezone().isoformat(timespec="seconds")}
                        self.active[aid] = alert
                        self.log.add("ALERT", alert["message"], sev)
                        changes.append({"action": "raised", **alert})
                else:
                    try:
                        ok = clear(tele)
                    except (TypeError, KeyError):
                        ok = False
                    if ok:
                        alert = self.active.pop(aid)
                        self.log.add("ALERT", f"Cleared: {aid.replace('_', ' ')}",
                                     "info")
                        changes.append({"action": "cleared", **alert})
        return changes

    def snapshot(self):
        with self._lock:
            return list(self.active.values())


# ---------------------------------------------------------------------------
# Telemetry hub — one producer loop, many SSE subscribers
# ---------------------------------------------------------------------------
class TelemetryHub:
    def __init__(self):
        self._subs = set()
        self._lock = threading.Lock()
        self.latest = None

    def subscribe(self):
        q = queue.Queue(maxsize=100)
        with self._lock:
            self._subs.add(q)
        return q

    def unsubscribe(self, q):
        with self._lock:
            self._subs.discard(q)

    def publish(self, payload):
        self.latest = payload
        with self._lock:
            dead = []
            for q in self._subs:
                try:
                    q.put_nowait(payload)
                except queue.Full:
                    dead.append(q)
            for q in dead:
                self._subs.discard(q)


log = MissionLog()
alerts = AlertEngine(log)
hub = TelemetryHub()
sim = UAVSimulator(tick_hz=TICK_HZ)
mav_listener = None

if SOURCE == "mavlink":
    from mavlink_listener import MAVLinkListener
    mav_listener = MAVLinkListener()
    mav_listener.start()
    log.add("SYSTEM", "GCS started — MAVLink source on udpin:0.0.0.0:14550")
else:
    log.add("SYSTEM", "GCS started — internal simulator source")

SEV_FOR = {"ARM": "info", "DISARM": "info", "MODE": "info", "NAV": "info",
           "WAYPOINT": "success", "MISSION": "info", "FAILSAFE": "critical",
           "CONNECTION": "warning"}


def producer_loop():
    """Single loop: advance source, run alerts, publish to subscribers."""
    period = 1.0 / TICK_HZ
    last_link = True
    while True:
        t0 = time.time()

        if SOURCE == "mavlink":
            tele = mav_listener.snapshot()
            events = mav_listener.drain_events()
            if tele is None:
                tele = {"ts": time.time(), "link_ok": False, "mode": "N/A",
                        "armed": False}
            if tele.get("link_ok") != last_link:
                last_link = tele.get("link_ok")
                log.add("CONNECTION",
                        "Telemetry link established" if last_link
                        else "Telemetry link lost",
                        "info" if last_link else "warning")
        else:
            tele = sim.step()
            tele["link_ok"] = True
            events = sim.drain_events()

        for etype, msg in events:
            log.add(etype, msg, SEV_FOR.get(etype, "info"))

        alert_changes = alerts.evaluate(tele)
        payload = {
            "telemetry": tele,
            "alerts": alerts.snapshot(),
            "alert_changes": alert_changes,
            "log_tail": log.all()[-1:] if events or alert_changes else [],
        }
        hub.publish(payload)

        time.sleep(max(0.0, period - (time.time() - t0)))


threading.Thread(target=producer_loop, daemon=True).start()


# ---------------------------------------------------------------------------
# Routes
# ---------------------------------------------------------------------------
@app.route("/")
def index():
    return send_from_directory(FRONTEND_DIR, "index.html")


@app.route("/stream")
def stream():
    def gen():
        q = hub.subscribe()
        try:
            # send latest immediately so UI paints fast
            if hub.latest:
                yield f"data: {json.dumps(hub.latest)}\n\n"
            while True:
                try:
                    payload = q.get(timeout=15.0)
                    yield f"data: {json.dumps(payload)}\n\n"
                except queue.Empty:
                    yield ": keepalive\n\n"
        finally:
            hub.unsubscribe(q)

    return Response(gen(), mimetype="text/event-stream",
                    headers={"Cache-Control": "no-cache",
                             "X-Accel-Buffering": "no"})


@app.route("/api/status")
def api_status():
    return jsonify({"source": SOURCE, "latest": hub.latest,
                    "log_size": len(log.all())})


@app.route("/api/waypoints", methods=["POST"])
def api_waypoints():
    if SOURCE == "mavlink":
        return jsonify({"ok": False,
                        "error": "Mission upload to MAVLink vehicle not enabled "
                                 "in this build — use sim mode"}), 400
    data = request.get_json(silent=True) or {}
    wps = data.get("waypoints", [])
    if not isinstance(wps, list) or not wps:
        return jsonify({"ok": False, "error": "waypoints must be a non-empty list"}), 400
    for w in wps:
        try:
            lat, lon = float(w["lat"]), float(w["lon"])
        except (KeyError, TypeError, ValueError):
            return jsonify({"ok": False, "error": "each waypoint needs numeric lat/lon"}), 400
        if not (-90 <= lat <= 90 and -180 <= lon <= 180):
            return jsonify({"ok": False, "error": f"waypoint out of range: {lat},{lon}"}), 400
        alt = w.get("alt", 60)
        try:
            alt = float(alt)
        except (TypeError, ValueError):
            return jsonify({"ok": False, "error": "alt must be numeric"}), 400
        if not (5 <= alt <= 400):
            return jsonify({"ok": False,
                            "error": f"altitude {alt} m outside safe envelope (5-400 m)"}), 400
    ok, msg = sim.set_waypoints(wps)
    return jsonify({"ok": ok, "message": msg})


@app.route("/api/waypoints", methods=["DELETE"])
def api_waypoints_clear():
    if SOURCE == "mavlink":
        return jsonify({"ok": False, "error": "not available in mavlink mode"}), 400
    sim.clear_waypoints()
    return jsonify({"ok": True, "message": "Mission cleared"})


@app.route("/api/command/<cmd>", methods=["POST"])
def api_command(cmd):
    if SOURCE == "mavlink":
        return jsonify({"ok": False,
                        "error": "Command uplink not enabled in mavlink mode"}), 400
    cmd = cmd.lower()
    if cmd == "takeoff":
        ok, msg = sim.arm_and_takeoff()
    elif cmd in ("auto", "manual", "hold", "rth", "land"):
        ok, msg = sim.set_mode(cmd)
    else:
        return jsonify({"ok": False, "error": f"unknown command '{cmd}'"}), 400
    return (jsonify({"ok": ok, "message": msg})
            if ok else (jsonify({"ok": False, "error": msg}), 409))


@app.route("/api/log")
def api_log():
    return jsonify(log.all())


@app.route("/api/log/export")
def api_log_export():
    fmt = request.args.get("format", "txt").lower()
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    if fmt == "csv":
        return Response(log.as_csv(), mimetype="text/csv",
                        headers={"Content-Disposition":
                                 f"attachment; filename=xo11_mission_log_{stamp}.csv"})
    return Response(log.as_txt(), mimetype="text/plain",
                    headers={"Content-Disposition":
                             f"attachment; filename=xo11_mission_log_{stamp}.txt"})


if __name__ == "__main__":
    port = int(os.environ.get("PORT", 5000))
    print(f"XO11 GCS running on http://localhost:{port}  (source: {SOURCE})")
    # threaded=True required: SSE holds a worker per client
    app.run(host="0.0.0.0", port=port, threaded=True, debug=False)

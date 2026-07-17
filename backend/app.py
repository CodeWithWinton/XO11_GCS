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

from simulator import PROFILES, UAVSimulator, haversine_m

FRONTEND_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "..", "frontend")
TICK_HZ = 4.0
SOURCE = os.environ.get("GCS_SOURCE", "sim").lower()


def _env_float(name, default, lo, hi):
    """Read a float env var defensively — bad values fall back to default."""
    try:
        v = float(os.environ.get(name, default))
        return min(hi, max(lo, v))
    except (TypeError, ValueError):
        return default


# GCS_BATT_ACCEL: speeds up battery drain (sim only) so a short demo video
# can show the LOW -> CRITICAL -> auto-RTH cascade. 1.0 = realistic.
BATT_ACCEL = _env_float("GCS_BATT_ACCEL", 1.0, 0.1, 500.0)
# GCS_GEOFENCE_M: max allowed distance from home before breach alert + RTH.
GEOFENCE_M = _env_float("GCS_GEOFENCE_M", 1500.0, 50.0, 50000.0)
MAX_WAYPOINTS = 100

app = Flask(__name__, static_folder=None)


@app.after_request
def add_cors(resp):
    # allow a separately-hosted frontend (e.g. Vercel static) to reach the API
    resp.headers["Access-Control-Allow-Origin"] = "*"
    resp.headers["Access-Control-Allow-Methods"] = "GET, POST, DELETE, OPTIONS"
    resp.headers["Access-Control-Allow-Headers"] = "Content-Type"
    return resp

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

    def clear(self):
        with self._lock:
            self._entries.clear()

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
def _num(tele, key, default):
    """Numeric-safe getter: MAVLink sources may report None for unknown
    values (e.g. battery_remaining = -1 -> None). Treat those as 'unknown',
    never as zero — otherwise we'd raise false CRITICAL alerts."""
    v = tele.get(key, default)
    if v is None or not isinstance(v, (int, float)):
        return default
    if isinstance(v, float) and (v != v or v in (float("inf"), float("-inf"))):
        return default
    return v


class AlertEngine:
    """
    Each rule: (id, severity, trigger_fn, clear_fn, message_fn, persist_s)
    Two anti-flap mechanisms:
      - hysteresis: trigger and clear thresholds differ
      - persistence: fast-fluctuating signals (GPS sats, RF) must stay bad
        for persist_s continuous seconds before the alert raises
    """

    def __init__(self, log):
        self.log = log
        self.active = {}    # id -> alert dict
        self._pending = {}  # id -> first time trigger became true
        self._lock = threading.Lock()
        self.rules = [
            ("BATTERY_CRITICAL", "critical",
             lambda t: _num(t, "battery_pct", 100) < 15,
             lambda t: _num(t, "battery_pct", 100) >= 17,
             lambda t: f"Battery CRITICAL: {_num(t, 'battery_pct', 0):.0f}% "
                       f"({_num(t, 'battery_voltage', 0):.1f} V) — land immediately",
             0.0),
            ("BATTERY_LOW", "warning",
             lambda t: _num(t, "battery_pct", 100) < 25,
             lambda t: _num(t, "battery_pct", 100) >= 27,
             lambda t: f"Battery LOW: {_num(t, 'battery_pct', 0):.0f}% "
                       f"({_num(t, 'battery_voltage', 0):.1f} V)",
             0.0),
            ("GPS_LOST", "warning",
             lambda t: _num(t, "sats", 99) < 6,
             lambda t: _num(t, "sats", 99) >= 7,
             lambda t: f"GPS degraded: only {_num(t, 'sats', 0):.0f} satellites locked",
             3.0),
            ("SIGNAL_WEAK", "warning",
             lambda t: _num(t, "signal_pct", 100) < 35,
             lambda t: _num(t, "signal_pct", 100) >= 45,
             lambda t: f"Signal WEAK: link quality {_num(t, 'signal_pct', 0):.0f}%",
             3.0),
            ("GEOFENCE_BREACH", "critical",
             lambda t: _num(t, "dist_home_m", 0) > _num(t, "geofence_m", float("inf")),
             lambda t: _num(t, "dist_home_m", 0) <= 0.9 * _num(t, "geofence_m", float("inf")),
             lambda t: f"GEOFENCE breach: {_num(t, 'dist_home_m', 0):.0f} m from home "
                       f"(limit {_num(t, 'geofence_m', 0):.0f} m)",
             0.0),
            ("LINK_LOST", "critical",
             lambda t: t.get("link_ok") is False,
             lambda t: t.get("link_ok", True) is True,
             lambda t: "Telemetry link LOST — no heartbeat",
             2.0),
        ]

    def evaluate(self, tele):
        """Returns list of newly raised/cleared alert events."""
        changes = []
        now = time.time()
        with self._lock:
            for aid, sev, trig, clear, msg, persist in self.rules:
                # BATTERY_LOW is superseded by BATTERY_CRITICAL
                if aid == "BATTERY_LOW" and ("BATTERY_CRITICAL" in self.active
                                             or _num(tele, "battery_pct", 100) < 15):
                    if aid in self.active:   # upgrade in progress: drop LOW
                        alert = self.active.pop(aid)
                        changes.append({"action": "cleared", **alert})
                    self._pending.pop(aid, None)
                    continue
                if aid not in self.active:
                    try:
                        fire = trig(tele)
                    except (TypeError, KeyError, ValueError):
                        fire = False
                    if not fire:
                        self._pending.pop(aid, None)
                        continue
                    # persistence gate: condition must hold continuously
                    first = self._pending.setdefault(aid, now)
                    if now - first < persist:
                        continue
                    self._pending.pop(aid, None)
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
                    except (TypeError, KeyError, ValueError):
                        ok = False
                    if ok:
                        alert = self.active.pop(aid)
                        self.log.add("ALERT", f"Cleared: {aid.replace('_', ' ')}",
                                     "info")
                        changes.append({"action": "cleared", **alert})
                    else:
                        # keep live values in the banner message
                        try:
                            self.active[aid]["message"] = msg(tele)
                        except (TypeError, KeyError, ValueError):
                            pass
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
sim = UAVSimulator(tick_hz=TICK_HZ, batt_accel=BATT_ACCEL, geofence_m=GEOFENCE_M)
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
           "CONNECTION": "warning", "WARNING": "warning"}


def producer_loop():
    """Single loop: advance source, run alerts, publish to subscribers.

    Hardened: any exception in one tick is logged and the loop continues —
    a telemetry glitch must never freeze the whole ground station."""
    period = 1.0 / TICK_HZ
    last_link = True
    consecutive_errors = 0
    while True:
        t0 = time.time()
        try:
            n_log_before = len(log.all())

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

            # send *all* log entries added this tick (events + alerts),
            # not just the last one
            new_entries = log.all()[n_log_before:]
            payload = {
                "telemetry": tele,
                "alerts": alerts.snapshot(),
                "alert_changes": alert_changes,
                "log_tail": new_entries,
            }
            hub.publish(payload)
            consecutive_errors = 0
        except Exception as exc:                          # noqa: BLE001
            consecutive_errors += 1
            if consecutive_errors <= 3 or consecutive_errors % 40 == 0:
                log.add("SYSTEM", f"Telemetry loop error: {exc!r}", "critical")
            time.sleep(0.5)

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
    if len(wps) > MAX_WAYPOINTS:
        return jsonify({"ok": False,
                        "error": f"too many waypoints ({len(wps)} > {MAX_WAYPOINTS})"}), 400
    import math as _m
    for i, w in enumerate(wps, 1):
        if not isinstance(w, dict):
            return jsonify({"ok": False, "error": f"waypoint {i} is not an object"}), 400
        try:
            lat, lon = float(w["lat"]), float(w["lon"])
            alt = float(w.get("alt", 60))
        except (KeyError, TypeError, ValueError):
            return jsonify({"ok": False,
                            "error": f"waypoint {i} needs numeric lat/lon/alt"}), 400
        if not all(_m.isfinite(v) for v in (lat, lon, alt)):
            return jsonify({"ok": False,
                            "error": f"waypoint {i} has non-finite coordinates"}), 400
        if not (-90 <= lat <= 90 and -180 <= lon <= 180):
            return jsonify({"ok": False,
                            "error": f"waypoint {i} out of range: {lat},{lon}"}), 400
        if not (5 <= alt <= sim.p_max_alt):
            return jsonify({"ok": False,
                            "error": f"waypoint {i} altitude {alt} m outside "
                                     f"{sim.p_label} envelope (5-{sim.p_max_alt:.0f} m)"}), 400
        d_home = haversine_m(sim.home_lat, sim.home_lon, lat, lon)
        if d_home > GEOFENCE_M:
            return jsonify({"ok": False,
                            "error": f"waypoint {i} is {d_home:.0f} m from home — "
                                     f"outside the {GEOFENCE_M:.0f} m geofence"}), 400
    ok, msg = sim.set_waypoints(wps)
    return jsonify({"ok": ok, "message": msg}) if ok else \
        (jsonify({"ok": False, "error": msg}), 400)


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
    elif cmd == "reset":
        ok, msg = sim.reset()
        if ok:
            # clear any latched alerts so the fresh flight starts clean
            with alerts._lock:
                alerts.active.clear()
                alerts._pending.clear()
            log.clear()
    elif cmd in ("auto", "manual", "hold", "rth", "land"):
        ok, msg = sim.set_mode(cmd)
    else:
        return jsonify({"ok": False, "error": f"unknown command '{cmd}'"}), 400
    return (jsonify({"ok": ok, "message": msg})
            if ok else (jsonify({"ok": False, "error": msg}), 409))


@app.route("/api/profiles")
def api_profiles():
    return jsonify({
        "current": sim.profile,
        "profiles": [{"id": k, "label": p["label"], "desc": p["desc"],
                      "cells": p["cell_count"], "pack_wh": p["pack_wh"],
                      "cruise_kmh": p["cruise_kmh"], "max_alt": p["max_alt"]}
                     for k, p in PROFILES.items()],
    })


@app.route("/api/profiles/<name>", methods=["POST"])
def api_set_profile(name):
    if SOURCE == "mavlink":
        return jsonify({"ok": False,
                        "error": "Airframe selection is sim-only — a real vehicle "
                                 "reports its own hardware"}), 400
    ok, msg = sim.set_profile(name)
    return (jsonify({"ok": True, "message": msg})
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

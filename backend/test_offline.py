"""Offline test harness — validates simulator physics, alert engine, and log
without needing Flask installed (stubs it out). Run: python3 test_offline.py"""
import sys, types, time, math

# ---- stub flask so app.py imports cleanly ----
flask = types.ModuleType("flask")
class _Fake:
    def __init__(self, *a, **k): pass
    def route(self, *a, **k): return lambda f: f
    def after_request(self, f): return f
    def run(self, *a, **k): pass
flask.Flask = _Fake
flask.Response = _Fake
flask.jsonify = lambda *a, **k: a
flask.request = types.SimpleNamespace(get_json=lambda **k: {}, args={})
flask.send_from_directory = lambda *a, **k: None
sys.modules["flask"] = flask

import simulator as simmod
from simulator import UAVSimulator, soc_to_cell_v, haversine_m
from app import AlertEngine, MissionLog

FAIL = []
def check(name, cond, detail=""):
    status = "PASS" if cond else "FAIL"
    print(f"[{status}] {name} {detail}")
    if not cond: FAIL.append(name)

# ================= battery curve calibration =================
check("SOC curve full = 4.20 V/cell", abs(soc_to_cell_v(1.0) - 4.20) < 1e-6)
check("SOC curve empty = 3.30 V/cell", abs(soc_to_cell_v(0.0) - 3.30) < 1e-6)
check("SOC curve mid ~3.82 V/cell", abs(soc_to_cell_v(0.5) - 3.82) < 0.01)
mono = all(soc_to_cell_v(a/100) >= soc_to_cell_v((a-1)/100) for a in range(1, 101))
check("SOC curve monotonic", mono)
check("SOC clamps out-of-range", soc_to_cell_v(2.0) == 4.20 and soc_to_cell_v(-1) == 3.30)

# ================= haversine sanity =================
d = haversine_m(12.9716, 77.5946, 12.9816, 77.5946)  # 0.01 deg lat ~ 1111.9m
check("haversine ~1112 m per 0.01deg lat", abs(d - 1111.9) < 5, f"got {d:.1f}")

# ================= full mission run =================
sim = UAVSimulator(tick_hz=4)
sim._t = time.time() - 0.25   # prime dt

ok, msg = sim.set_mode("AUTO")
check("AUTO rejected when disarmed", not ok, f"({msg})")
ok, msg = sim.arm_and_takeoff()
check("takeoff accepted from IDLE", ok)
ok, msg = sim.arm_and_takeoff()
check("double takeoff rejected", not ok)

sim.set_waypoints([
    {"lat": sim.home_lat + 0.003, "lon": sim.home_lon + 0.002, "alt": 60},
    {"lat": sim.home_lat + 0.002, "lon": sim.home_lon + 0.005, "alt": 80},
])

log = MissionLog()
alerts = AlertEngine(log)

# simulate with 0.25 s virtual ticks (monkeypatch time)
t_virtual = time.time()
events_seen = set()
max_alt = 0; max_gs = 0
landed_tick = None
orig_time = time.time
try:
    for i in range(12000):  # up to 50 virtual minutes
        t_virtual += 0.25
        simmod.time.time = lambda: t_virtual
        snap = sim.step()
        max_alt = max(max_alt, snap["alt"])
        max_gs = max(max_gs, snap["groundspeed"])
        for et, m in sim.drain_events():
            events_seen.add(et)
        if snap["mode"] == "IDLE" and i > 100:
            landed_tick = i
            break
finally:
    simmod.time.time = orig_time

check("takeoff reached cruise alt", max_alt >= 59, f"max_alt={max_alt:.1f}")
check("altitude bounded (<=120m)", max_alt <= 120, f"max_alt={max_alt:.1f}")
check("groundspeed bounded (<=90km/h)", max_gs <= 90, f"max_gs={max_gs:.1f}")
check("waypoints reached", "WAYPOINT" in events_seen)
check("mission -> RTH -> land completed", landed_tick is not None,
      f"landed at tick {landed_tick}")
check("mode/nav events logged", {"MODE", "NAV", "DISARM"} <= events_seen,
      f"events={events_seen}")
final = sim.snapshot()
check("returned near home", final["dist_home_m"] < 30, f"{final['dist_home_m']} m")
check("battery drained plausibly", 50 < final["battery_pct"] < 100,
      f"{final['battery_pct']}% after flight")
check("voltage in 6S range", 19.8 <= final["battery_voltage"] <= 25.4,
      f"{final['battery_voltage']} V")

# ================= alert engine cases =================
def tele(**kw):
    base = {"battery_pct": 80, "battery_voltage": 23.0, "sats": 14,
            "signal_pct": 90, "link_ok": True}
    base.update(kw); return base

a = AlertEngine(MissionLog())
ch = a.evaluate(tele(battery_pct=24))
check("LOW raised at 24%", any(c["id"] == "BATTERY_LOW" and c["action"] == "raised" for c in ch))
ch = a.evaluate(tele(battery_pct=25.5))
check("LOW holds in hysteresis band (25.5%)", not ch and len(a.snapshot()) == 1)
ch = a.evaluate(tele(battery_pct=28))
check("LOW cleared at 28%", any(c["action"] == "cleared" for c in ch))

a2 = AlertEngine(MissionLog())
ch = a2.evaluate(tele(battery_pct=14))
ids = {c["id"] for c in ch if c["action"] == "raised"}
check("CRITICAL raised at 14%", "BATTERY_CRITICAL" in ids)
check("LOW suppressed when CRITICAL", "BATTERY_LOW" not in ids)

# --- persistence/debounce tests use a controllable clock ---
CLK = {"t": 1000.0}
_real_time_fn = time.time
time.time = lambda: CLK["t"]
try:
    a3 = AlertEngine(MissionLog())
    a3.evaluate(tele(sats=5))
    check("GPS_LOST not raised instantly (3s debounce)", not a3.snapshot())
    CLK["t"] += 3.1
    a3.evaluate(tele(sats=5))
    check("GPS_LOST raised after sustained 3s", len(a3.snapshot()) == 1)
    a3.evaluate(tele(sats=6))
    check("GPS holds at 6 sats (hysteresis)", len(a3.snapshot()) == 1)
    a3.evaluate(tele(sats=8))
    check("GPS cleared at 8 sats", len(a3.snapshot()) == 0)

    a3b = AlertEngine(MissionLog())
    a3b.evaluate(tele(sats=5)); CLK["t"] += 1.0
    a3b.evaluate(tele(sats=9)); CLK["t"] += 3.0     # blip recovers -> reset
    a3b.evaluate(tele(sats=5))
    check("brief GPS blip never raises (no flapping)", not a3b.snapshot())

    a4 = AlertEngine(MissionLog())
    a4.evaluate(tele(signal_pct=30)); CLK["t"] += 3.1
    a4.evaluate(tele(signal_pct=30))
    check("SIGNAL_WEAK raised after sustained 3s", len(a4.snapshot()) == 1)
    a4.evaluate(tele(signal_pct=40))
    check("SIGNAL holds at 40% (hysteresis)", len(a4.snapshot()) == 1)
    a4.evaluate(tele(signal_pct=50))
    check("SIGNAL cleared at 50%", len(a4.snapshot()) == 0)

    a5 = AlertEngine(MissionLog())
    a5.evaluate(tele(link_ok=False)); CLK["t"] += 2.1
    a5.evaluate(tele(link_ok=False))
    check("LINK_LOST raised after 2s", len(a5.snapshot()) == 1)
    a5.evaluate({"link_ok": False})   # missing keys must not crash
    check("alert engine survives sparse telemetry", True)
finally:
    time.time = _real_time_fn

# ---- hardening regression cases ----
a6 = AlertEngine(MissionLog())
ch = a6.evaluate(tele(battery_pct=None, battery_voltage=None))
check("None battery -> no crash, no false alert", ch == [] and not a6.snapshot())
ch = a6.evaluate(tele(battery_pct=-1))   # raw MAVLink 'unknown' sentinel
check("-1% battery raises CRITICAL (listener maps to None upstream)",
      any(c["id"] == "BATTERY_CRITICAL" for c in ch))

a7 = AlertEngine(MissionLog())
a7.evaluate(tele(battery_pct=20))                 # LOW active
ch = a7.evaluate(tele(battery_pct=12))            # drops to CRITICAL
ids_active = {x["id"] for x in a7.snapshot()}
check("LOW auto-cleared when CRITICAL raises",
      ids_active == {"BATTERY_CRITICAL"},
      f"active={ids_active}")

a8 = AlertEngine(MissionLog())
ch = a8.evaluate(tele(dist_home_m=1600, geofence_m=1500))
check("GEOFENCE_BREACH raised past fence", any(c["id"] == "GEOFENCE_BREACH" for c in ch))
a8.evaluate(tele(dist_home_m=1400, geofence_m=1500))
check("GEOFENCE holds in hysteresis band (1400m)", len(a8.snapshot()) == 1)
a8.evaluate(tele(dist_home_m=1200, geofence_m=1500))
check("GEOFENCE cleared at 0.8x fence", len(a8.snapshot()) == 0)
a9 = AlertEngine(MissionLog())
ch = a9.evaluate(tele(dist_home_m=5000))          # no geofence_m in telemetry
check("no geofence field -> rule never fires", not a9.snapshot())

a10 = AlertEngine(MissionLog())
a10.evaluate(tele(battery_pct=20))
a10.evaluate(tele(battery_pct=19, battery_voltage=22.1))
check("active alert message refreshes with live values",
      "19%" in a10.snapshot()[0]["message"])

# ================= log export =================
ml = MissionLog()
ml.add("TEST", 'msg with, comma and "quotes"', "warning")
txt = ml.as_txt(); csvs = ml.as_csv()
check("txt export contains entry", "TEST" in txt and "WARNING" in txt)
check("csv export escapes quotes", '""quotes""' in csvs)
check("csv has header", csvs.startswith("ts,type,severity,message"))

# ================= waypoint validation logic (mirror of API) =================
sim2 = UAVSimulator()
ok, _ = sim2.set_waypoints([{"lat": 12.98, "lon": 77.60}])
check("waypoint default alt applied", ok and sim2.waypoints[0]["alt"] == 60.0)

# ================= failsafe =================
sim3 = UAVSimulator(tick_hz=4)
sim3.arm_and_takeoff()
sim3.soc = 0.14
sim3._t = time.time() - 0.25
sim3.step()
evs = [m for _, m in sim3.drain_events()]
check("battery-critical failsafe -> RTH", sim3.mode == "RTH",
      f"mode={sim3.mode}")
sim4 = UAVSimulator(); sim4.batt_pct = 18
ok, msg = sim4.arm_and_takeoff()
check("takeoff refused below 20% battery", not ok, f"({msg})")

# ================= new hardening: simulator =================
# geofence failsafe
sf = UAVSimulator(tick_hz=4, geofence_m=200.0)
sf.arm_and_takeoff()
sf.mode = "MANUAL"; sf.alt = 60
sf.lat, _ = sf.home_lat + 0.003, 0   # ~330 m north of home
sf._t = time.time() - 0.25
sf.step()
check("geofence breach forces RTH", sf.mode == "RTH", f"mode={sf.mode}")
evs = [m for _, m in sf.drain_events()]
check("geofence failsafe logged", any("Geofence" in m for m in evs))
# operator RTH is respected (no re-trigger spam): stays RTH
sf._t = time.time() - 0.25; sf.step()
check("no repeat geofence event while returning",
      not any("Geofence" in m for _, m in sf.drain_events()))

# waypoint hardening
sw = UAVSimulator()
ok, msg = sw.set_waypoints([{"lat": float("nan"), "lon": 77.6}])
check("NaN waypoint rejected", not ok, f"({msg})")
ok, msg = sw.set_waypoints([{"lat": 12.98}])
check("missing lon rejected", not ok)
ok, msg = sw.set_waypoints([{"lat": 12.98, "lon": 77.6}] * 101)
check(">100 waypoints rejected", not ok)

# mission feasibility warning
sm = UAVSimulator()
sm.soc = 0.30   # low battery, long mission
sm.set_waypoints([{"lat": sm.home_lat + 0.30, "lon": sm.home_lon}])  # ~33 km
warn = [m for _, m in sm.drain_events()]
check("infeasible mission warned", any("exceeds estimated range" in m for m in warn),
      str(warn))

# batt_accel drains faster but stays consistent
b1 = UAVSimulator(tick_hz=4, batt_accel=50.0)
b1.arm_and_takeoff()
t_v = time.time()
orig = simmod.time.time
try:
    for _ in range(400):   # 100 virtual seconds
        t_v += 0.25
        simmod.time.time = lambda: t_v
        b1.step()
finally:
    simmod.time.time = orig
check("batt_accel=50 drains visibly in 100s", b1.batt_pct < 95,
      f"{b1.batt_pct}%")
check("accelerated flight-time estimate scales down",
      (b1.flight_time_remaining_s() or 0) < 600,
      f"{b1.flight_time_remaining_s()}s")
check("exhausted battery forces LAND/disarm",
      b1.soc > 0 or b1.mode in ("LAND", "IDLE"),
      f"soc={b1.soc:.2f} mode={b1.mode}")

# snapshot exposes geofence for the UI
check("snapshot carries geofence_m", UAVSimulator().snapshot()["geofence_m"] == 1500.0)

# ================= reset =================
sr = UAVSimulator(tick_hz=4)
sr.arm_and_takeoff()
sr.alt = 50.0
ok, msg = sr.reset()
check("reset refused mid-flight", not ok, f"({msg})")
sr.alt = 0.0; sr.armed = False
sr.soc = 0.4; sr.set_waypoints([{"lat": sr.home_lat + 0.001, "lon": sr.home_lon}])
sr.lat = sr.home_lat + 0.01
ok, msg = sr.reset()
snap = sr.snapshot()
check("reset restores full battery", ok and snap["battery_pct"] == 100.0)
check("reset returns UAV to home", snap["lat"] == sr.home_lat and snap["dist_home_m"] == 0.0)
check("reset clears mission", snap["wp_total"] == 0)
check("reset -> IDLE disarmed", snap["mode"] == "IDLE" and not snap["armed"])
check("reset logged", any("reset" in m.lower() for _, m in sr.drain_events()))

print()
if FAIL:
    print(f"{len(FAIL)} FAILURES: {FAIL}"); sys.exit(1)
print("ALL TESTS PASSED")

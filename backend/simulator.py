"""
XO11 UAV Systems — Ground Control Station
Flight Simulator (calibrated physics model)

Simulates a quadcopter-class UAV with:
  - Flight state machine: IDLE -> TAKEOFF -> AUTO / MANUAL / HOLD -> RTH -> LAND
  - 6S Li-ion battery with realistic discharge curve + load-dependent voltage sag
  - GPS satellite count fluctuation (Poisson-ish jitter, occasional degradation)
  - RF signal strength model (log-distance path loss + fading)
  - Wind gusts affecting airspeed vs ground speed
  - Waypoint navigation with arrival detection

All units:
  altitude  : meters (AGL)
  airspeed  : km/h
  groundspeed: km/h
  battery_voltage : volts (6S: 25.2V full -> 19.8V empty)
  distances : meters
"""

import math
import random
import time
import threading

EARTH_R = 6371000.0  # meters

# ---------------------------------------------------------------------------
# Calibration constants (XO11 quad, 6S 22Ah pack)
# ---------------------------------------------------------------------------
CELL_COUNT = 6
CELL_FULL_V = 4.20          # per-cell full charge
CELL_EMPTY_V = 3.30         # per-cell cutoff
PACK_CAPACITY_WH = 480.0    # ~22Ah * 21.7V nominal
HOVER_POWER_W = 320.0       # hover draw
CRUISE_POWER_W = 410.0      # cruise draw at CRUISE_SPEED
CLIMB_POWER_W = 560.0       # full climb draw
INTERNAL_R = 0.012 * CELL_COUNT  # pack internal resistance (ohms) -> sag

CRUISE_SPEED_KMH = 45.0
MAX_SPEED_KMH = 68.0
CLIMB_RATE = 2.5            # m/s
DESCENT_RATE = 1.8          # m/s
CRUISE_ALT = 60.0           # m
WAYPOINT_RADIUS = 12.0      # m arrival threshold
TURN_RATE_DEG_S = 45.0

# Li-ion discharge curve: (state-of-charge fraction, per-cell open-circuit V)
SOC_CURVE = [
    (1.00, 4.20), (0.95, 4.13), (0.90, 4.06), (0.80, 3.98),
    (0.70, 3.92), (0.60, 3.87), (0.50, 3.82), (0.40, 3.78),
    (0.30, 3.73), (0.20, 3.66), (0.15, 3.60), (0.10, 3.52),
    (0.05, 3.42), (0.02, 3.35), (0.00, 3.30),
]


def soc_to_cell_v(soc):
    """Interpolate per-cell open-circuit voltage from state of charge."""
    soc = max(0.0, min(1.0, soc))
    for i in range(len(SOC_CURVE) - 1):
        s1, v1 = SOC_CURVE[i]
        s2, v2 = SOC_CURVE[i + 1]
        if s2 <= soc <= s1:
            t = (soc - s2) / (s1 - s2) if s1 != s2 else 0.0
            return v2 + t * (v1 - v2)
    return SOC_CURVE[-1][1]


def haversine_m(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_R * math.asin(math.sqrt(a))


def bearing_deg(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    x = math.sin(dl) * math.cos(p2)
    y = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0


def offset_latlon(lat, lon, dist_m, brg_deg):
    """Move dist_m meters along bearing from (lat, lon)."""
    br = math.radians(brg_deg)
    dlat = (dist_m * math.cos(br)) / EARTH_R
    dlon = (dist_m * math.sin(br)) / (EARTH_R * math.cos(math.radians(lat)))
    return lat + math.degrees(dlat), lon + math.degrees(dlon)


class UAVSimulator:
    """Thread-safe UAV flight simulator. Call step() at ~2-5 Hz."""

    MODES = ("IDLE", "TAKEOFF", "AUTO", "MANUAL", "HOLD", "RTH", "LAND")

    def __init__(self, home_lat=12.9716, home_lon=77.5946, tick_hz=4.0,
                 batt_accel=1.0, geofence_m=1500.0):
        self._lock = threading.RLock()
        self.tick = 1.0 / tick_hz
        # batt_accel > 1 accelerates battery drain (demo videos);
        # physics stays otherwise identical.
        self.batt_accel = max(0.1, float(batt_accel))
        self.geofence_m = max(50.0, float(geofence_m))

        self.home_lat, self.home_lon = home_lat, home_lon
        self.lat, self.lon = home_lat, home_lon
        self.alt = 0.0
        self.heading = 0.0
        self.airspeed = 0.0
        self.groundspeed = 0.0
        self.climb = 0.0

        self.mode = "IDLE"
        self.armed = False
        self.soc = 1.0
        self.batt_v = soc_to_cell_v(1.0) * CELL_COUNT
        self.batt_pct = 100.0
        self.current_a = 0.0
        self.sats = 14
        self.hdop = 0.8
        self.signal = 99.0     # percent link quality
        self.rssi_dbm = -42.0

        self.waypoints = []    # list of dicts {lat, lon, alt}
        self.wp_index = 0

        # smoothed power draw for a stable flight-time estimate
        self._power_ema = None
        self._fence_breached = False

        # episodic GPS model state
        self._gps_episode_t = 0.0
        self._gps_episode_sats = 4
        self._gps_healthy_sats = 14
        self._gps_move_acc = 0.0

        # wind
        self._wind_dir = random.uniform(0, 360)
        self._wind_kmh = random.uniform(4, 12)
        self._gust = 0.0

        self._events = []      # (event_type, message) pairs emitted since last drain
        self._t = time.time()

    # ---------------- public control API ----------------

    def arm_and_takeoff(self):
        with self._lock:
            if self.mode != "IDLE":
                return False, "UAV is not idle"
            if self.batt_pct < 20:
                return False, "Battery too low for takeoff"
            self.armed = True
            self._set_mode("TAKEOFF")
            self._event("ARM", "UAV armed — takeoff initiated")
            return True, "Takeoff initiated"

    def set_mode(self, mode):
        with self._lock:
            mode = mode.upper()
            if mode not in self.MODES:
                return False, f"Unknown mode {mode}"
            if not self.armed and mode not in ("IDLE",):
                return False, "UAV is disarmed — take off first"
            if mode == "AUTO" and not self.waypoints:
                return False, "No mission uploaded — add waypoints first"
            self._set_mode(mode)
            return True, f"Mode set to {mode}"

    def set_waypoints(self, wps):
        with self._lock:
            if len(wps) > 100:
                return False, "Mission rejected — more than 100 waypoints"
            try:
                parsed = [
                    {"lat": float(w["lat"]), "lon": float(w["lon"]),
                     "alt": float(w.get("alt", CRUISE_ALT))}
                    for w in wps
                ]
            except (KeyError, TypeError, ValueError):
                return False, "Mission rejected — malformed waypoint"
            for w in parsed:
                if not all(math.isfinite(v) for v in (w["lat"], w["lon"], w["alt"])):
                    return False, "Mission rejected — non-finite waypoint"
            self.waypoints = parsed
            self.wp_index = 0
            self._event("MISSION", f"Mission uploaded — {len(self.waypoints)} waypoints")

            # ---- feasibility check: mission length + return leg vs range ----
            total_m = 0.0
            prev = (self.lat, self.lon)
            for w in parsed:
                total_m += haversine_m(prev[0], prev[1], w["lat"], w["lon"])
                prev = (w["lat"], w["lon"])
            total_m += haversine_m(prev[0], prev[1], self.home_lat, self.home_lon)
            usable_wh = max(0.0, (self.soc - 0.15) * PACK_CAPACITY_WH) / self.batt_accel
            range_m = usable_wh / CRUISE_POWER_W * 3600.0 * (CRUISE_SPEED_KMH / 3.6)
            if total_m > range_m:
                self._event("WARNING",
                            f"Mission length {total_m/1000:.1f} km exceeds estimated "
                            f"range {range_m/1000:.1f} km at current battery — "
                            f"expect battery failsafe mid-mission")
            return True, f"{len(self.waypoints)} waypoints accepted"

    def clear_waypoints(self):
        with self._lock:
            self.waypoints = []
            self.wp_index = 0
            self._event("MISSION", "Mission cleared")
            if self.mode == "AUTO":
                self._set_mode("HOLD")

    def reset(self):
        """Full simulation reset: UAV back at home, fresh pack, mission cleared.
        Refuses mid-flight — land or RTH first (mirrors real ops discipline)."""
        with self._lock:
            if self.armed and self.alt > 0.5:
                return False, "Cannot reset mid-flight — LAND or RTH first"
            self.lat, self.lon = self.home_lat, self.home_lon
            self.alt = 0.0
            self.heading = 0.0
            self.airspeed = self.groundspeed = self.climb = 0.0
            self.mode = "IDLE"
            self.armed = False
            self.soc = 1.0
            self.batt_v = soc_to_cell_v(1.0) * CELL_COUNT
            self.batt_pct = 100.0
            self.current_a = 0.0
            self._power_ema = None
            self._fence_breached = False
            self.waypoints = []
            self.wp_index = 0
            self.sats = 14
            self.hdop = 0.8
            self._gps_episode_t = 0.0
            self._gps_healthy_sats = 14
            self._event("SYSTEM", "Simulation reset — battery 100%, UAV at home, mission cleared")
            return True, "Simulation reset"

    def drain_events(self):
        with self._lock:
            ev, self._events = self._events, []
            return ev

    # ---------------- internals ----------------

    def _set_mode(self, mode):
        if mode != self.mode:
            self._event("MODE", f"Flight mode changed: {self.mode} -> {mode}")
            self.mode = mode

    def _event(self, etype, msg):
        self._events.append((etype, msg))

    # ---------------- physics step ----------------

    def step(self):
        with self._lock:
            now = time.time()
            dt = min(now - self._t, 1.0)
            self._t = now
            if dt <= 0:
                return self.snapshot()

            self._step_wind(dt)
            self._step_flight(dt)
            self._step_battery(dt)
            self._step_gps(dt)
            self._step_signal(dt)
            self._step_geofence()
            return self.snapshot()

    def _step_geofence(self):
        """Geofence: warn-and-return failsafe. Operator RTH/LAND is respected."""
        if not self.armed or self.mode in ("RTH", "LAND", "IDLE"):
            self._fence_breached = False
            return
        d = haversine_m(self.lat, self.lon, self.home_lat, self.home_lon)
        if d > self.geofence_m and not self._fence_breached:
            self._fence_breached = True
            self._event("FAILSAFE",
                        f"Geofence breach at {d:.0f} m — automatic Return-To-Home engaged")
            self._set_mode("RTH")
        elif d <= 0.9 * self.geofence_m:
            self._fence_breached = False

    def _step_wind(self, dt):
        # slowly wandering wind + gust envelope
        self._wind_dir = (self._wind_dir + random.gauss(0, 2) * dt) % 360
        self._wind_kmh = max(0.0, min(25.0, self._wind_kmh + random.gauss(0, 0.6) * dt))
        # gusts decay, occasionally spike
        self._gust *= math.exp(-dt / 3.0)
        if random.random() < 0.02 * dt * 4:
            self._gust = random.uniform(3, 9)

    def _wind_component_along(self, heading):
        """Tailwind (+) / headwind (-) component in km/h."""
        rel = math.radians(self._wind_dir - heading)
        return -(self._wind_kmh + self._gust) * math.cos(rel)

    def _step_flight(self, dt):
        target_speed = 0.0
        target_alt = self.alt
        target_hdg = self.heading

        if self.mode == "IDLE":
            self.airspeed = self.groundspeed = 0.0
            self.climb = 0.0
            return

        if self.mode == "TAKEOFF":
            target_alt = CRUISE_ALT
            target_speed = 0.0
            if self.alt >= CRUISE_ALT - 0.5:
                self._event("NAV", f"Takeoff complete — reached {CRUISE_ALT:.0f} m")
                self._set_mode("AUTO" if self.waypoints else "HOLD")

        elif self.mode == "HOLD":
            target_speed = 0.0
            target_alt = max(self.alt, 20.0)

        elif self.mode == "MANUAL":
            # gentle wandering in manual to keep demo alive
            target_speed = CRUISE_SPEED_KMH * 0.6
            target_hdg = (self.heading + random.gauss(0, 6) * dt) % 360
            target_alt = self.alt + random.gauss(0, 0.5)
            target_alt = max(15.0, min(120.0, target_alt))

        elif self.mode == "AUTO":
            if self.wp_index >= len(self.waypoints):
                self._event("NAV", "Mission complete — all waypoints reached, returning home")
                self._set_mode("RTH")
            else:
                wp = self.waypoints[self.wp_index]
                dist = haversine_m(self.lat, self.lon, wp["lat"], wp["lon"])
                target_hdg = bearing_deg(self.lat, self.lon, wp["lat"], wp["lon"])
                target_alt = wp["alt"]
                # slow down on approach
                target_speed = min(CRUISE_SPEED_KMH, max(12.0, dist / 4.0 * 3.6))
                if dist < WAYPOINT_RADIUS:
                    self._event("WAYPOINT",
                                f"Waypoint {self.wp_index + 1}/{len(self.waypoints)} reached")
                    self.wp_index += 1

        elif self.mode == "RTH":
            dist = haversine_m(self.lat, self.lon, self.home_lat, self.home_lon)
            if dist < WAYPOINT_RADIUS:
                self._event("NAV", "Arrived over home position — landing")
                self._set_mode("LAND")
            else:
                target_hdg = bearing_deg(self.lat, self.lon, self.home_lat, self.home_lon)
                target_alt = max(self.alt, 40.0)
                target_speed = min(CRUISE_SPEED_KMH, max(12.0, dist / 4.0 * 3.6))

        elif self.mode == "LAND":
            target_speed = 0.0
            target_alt = 0.0
            if self.alt <= 0.3:
                self.alt = 0.0
                self.armed = False
                self._event("DISARM", "Touchdown — UAV disarmed")
                self._set_mode("IDLE")
                return

        # -- heading: rate-limited turn toward target --
        err = (target_hdg - self.heading + 540) % 360 - 180
        max_turn = TURN_RATE_DEG_S * dt
        self.heading = (self.heading + max(-max_turn, min(max_turn, err))) % 360

        # -- speed: first-order response --
        self.airspeed += (target_speed - self.airspeed) * min(1.0, 0.8 * dt)
        self.airspeed = max(0.0, min(MAX_SPEED_KMH, self.airspeed))

        # ground speed = airspeed + wind component (never negative).
        # Exception: when commanded to hold position (HOLD/LAND/TAKEOFF with
        # zero target speed) the flight controller cancels wind drift — a
        # multirotor does not blow away while position-holding.
        if target_speed < 0.5 and self.airspeed < 2.0:
            self.groundspeed = 0.0
        else:
            self.groundspeed = max(0.0, self.airspeed
                                   + self._wind_component_along(self.heading))

        # -- altitude: rate-limited climb/descent --
        alt_err = target_alt - self.alt
        rate = CLIMB_RATE if alt_err > 0 else -DESCENT_RATE
        if abs(alt_err) < abs(rate) * dt:
            self.climb = alt_err / dt if dt > 0 else 0.0
            self.alt = target_alt
        else:
            self.climb = rate
            self.alt += rate * dt
        self.alt = max(0.0, self.alt)

        # -- position update from ground speed --
        if self.groundspeed > 0.1 and self.alt > 0.2:
            d = self.groundspeed / 3.6 * dt
            self.lat, self.lon = offset_latlon(self.lat, self.lon, d, self.heading)

    def _step_battery(self, dt):
        if not self.armed:
            power = 4.0  # avionics idle
        elif self.climb > 0.5:
            power = CLIMB_POWER_W
        elif self.airspeed > 5:
            # power grows with speed^2 above hover baseline
            frac = (self.airspeed / CRUISE_SPEED_KMH) ** 2
            power = HOVER_POWER_W + (CRUISE_POWER_W - HOVER_POWER_W) * min(1.6, frac)
        elif self.alt > 0.5:
            power = HOVER_POWER_W
        else:
            power = 4.0
        # headwind penalty
        head = -self._wind_component_along(self.heading)
        if head > 0 and self.airspeed > 5:
            power *= 1.0 + min(0.25, head / 100.0)

        self.soc = max(0.0, self.soc - (power * self.batt_accel * dt / 3600.0)
                       / PACK_CAPACITY_WH)

        # exponential moving average of power -> stable flight-time estimate
        if self._power_ema is None:
            self._power_ema = power
        else:
            self._power_ema += (power - self._power_ema) * min(1.0, dt / 10.0)

        ocv = soc_to_cell_v(self.soc) * CELL_COUNT
        nominal_v = 3.7 * CELL_COUNT
        self.current_a = power / max(nominal_v, ocv * 0.9)
        sag = self.current_a * INTERNAL_R
        self.batt_v = round(ocv - sag + random.gauss(0, 0.02), 2)
        self.batt_pct = round(self.soc * 100.0, 1)

        # auto-RTH failsafe at critical battery
        if self.armed and self.batt_pct < 15 and self.mode not in ("RTH", "LAND", "IDLE"):
            self._event("FAILSAFE", "Battery critical — automatic Return-To-Home engaged")
            self._set_mode("RTH")
        # battery exhausted: forced landing wherever we are
        if self.armed and self.soc <= 0.0 and self.mode != "LAND":
            self._event("FAILSAFE", "Battery exhausted — forced landing")
            self._set_mode("LAND")

    def _step_gps(self, dt):
        """Episodic GPS model: healthy (12-16 sats) most of the time, with
        rare degradation episodes lasting 15-40 s that dip to 3-5 sats.
        This mirrors real receivers (multipath under structures, ionospheric
        scintillation) and avoids threshold flapping in the alert system."""
        if self._gps_episode_t > 0:
            self._gps_episode_t -= dt
            target = self._gps_episode_sats
        else:
            target = self._gps_healthy_sats
            # occasionally re-pick the healthy baseline
            if random.random() < 0.01 * dt * 4:
                self._gps_healthy_sats = random.randint(12, 16)
            # ~once every few minutes: start a degradation episode
            if random.random() < 0.0015 * dt * 4:
                self._gps_episode_t = random.uniform(15, 40)
                self._gps_episode_sats = random.randint(3, 5)
        # move one satellite at a time toward the target (~2 sats/s max)
        self._gps_move_acc += dt
        if self._gps_move_acc >= 0.5:
            self._gps_move_acc = 0.0
            if self.sats < target:
                self.sats += 1
            elif self.sats > target:
                self.sats -= 1
            elif random.random() < 0.15:      # tiny jitter around baseline
                self.sats += random.choice([-1, 1])
        self.sats = int(max(3, min(18, self.sats)))
        self.hdop = round(max(0.5, min(6.0, 12.0 / max(4, self.sats)
                                       + random.gauss(0, 0.05))), 2)

    def _step_signal(self, dt):
        # log-distance path loss from home (ground antenna) + shadow fading
        d = max(5.0, haversine_m(self.lat, self.lon, self.home_lat, self.home_lon))
        rssi = -40.0 - 22.0 * math.log10(d / 5.0) + random.gauss(0, 1.5)
        self.rssi_dbm = round(max(-110.0, min(-35.0, rssi)), 1)
        # map RSSI [-110..-50] -> quality [0..100]
        q = (self.rssi_dbm + 110.0) / 60.0 * 100.0
        self.signal = round(max(0.0, min(100.0, q)), 1)

    # ---------------- output ----------------

    def flight_time_remaining_s(self):
        """Estimate seconds of flight remaining (reserve to 10% SOC).
        Uses EMA-smoothed power so the number doesn't jump every tick,
        and accounts for batt_accel so demo mode stays self-consistent."""
        if self.current_a < 1.0 or not self.armed:
            return None
        usable_wh = max(0.0, (self.soc - 0.10) * PACK_CAPACITY_WH)
        power = (self._power_ema or self.current_a * self.batt_v) * self.batt_accel
        if power <= 0:
            return None
        return int(usable_wh / power * 3600.0)

    def snapshot(self):
        with self._lock:
            dist_home = haversine_m(self.lat, self.lon, self.home_lat, self.home_lon)
            return {
                "ts": time.time(),
                "lat": round(self.lat, 7),
                "lon": round(self.lon, 7),
                "alt": round(self.alt, 1),
                "heading": round(self.heading, 1),
                "airspeed": round(self.airspeed, 1),
                "groundspeed": round(self.groundspeed, 1),
                "climb": round(self.climb, 2),
                "mode": self.mode,
                "armed": self.armed,
                "battery_voltage": self.batt_v,
                "battery_pct": self.batt_pct,
                "current_a": round(self.current_a, 1),
                "flight_time_remaining_s": self.flight_time_remaining_s(),
                "sats": self.sats,
                "hdop": self.hdop,
                "signal_pct": self.signal,
                "rssi_dbm": self.rssi_dbm,
                "home": {"lat": self.home_lat, "lon": self.home_lon},
                "wind_kmh": round(self._wind_kmh + self._gust, 1),
                "wind_dir": round(self._wind_dir, 0),
                "wp_index": self.wp_index,
                "wp_total": len(self.waypoints),
                "dist_home_m": round(dist_home, 1),
                "geofence_m": self.geofence_m,
            }

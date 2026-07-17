# XO11 GCS — Ground Control Station

XO11 UAV Systems' Ground Control Station software: a real-time dashboard for monitoring, mission-planning, and commanding the XO11 UAV. Built for the CS/IT team real-project assignment (July 2026).

![stack](https://img.shields.io/badge/backend-Python%20Flask-blue) ![stack](https://img.shields.io/badge/frontend-Leaflet%20%2B%20Chart.js-green) ![stack](https://img.shields.io/badge/protocol-MAVLink%20(pymavlink)-orange)

## Features (mapped to assignment modules)

**Module 1 — Live Map & UAV Position.** Interactive Leaflet map (OpenStreetMap tiles, no API key), heading-rotated UAV icon updating in real time, breadcrumb flight trail, home/launch marker, click-to-place waypoints with numbered markers, and a dashed planned-route line connecting them.

**Module 2 — Telemetry Dashboard.** Live altitude (m), climb rate, airspeed and ground speed (km/h — they differ because wind is simulated), battery voltage + percentage + current draw with a color-coded bar, estimated flight time remaining, GPS lat/lon, satellite count, HDOP, flight mode, 5-bar signal-strength indicator with RSSI in dBm, heading compass, wind, and distance-to-home. Everything streams at 4 Hz over Server-Sent Events.

**Module 3 — Alert & Safety System.** Battery LOW (<25%, amber) and CRITICAL (<15%, red + auto-RTH failsafe), GPS degraded (<6 satellites), signal WEAK (<35% link quality), and link-lost heartbeat detection. Every alert carries a timestamp, is logged, shows as an on-map banner, and plays an audio tone. Alerts use **hysteresis** (e.g. LOW trips below 25% but only clears above 27%) so a value hovering at a threshold doesn't spam the operator.

**Module 4 — Mission Log.** Scrolling timestamped log of waypoint arrivals, mode changes, alerts, arm/disarm, failsafes, and connection events. Exportable as `.txt` or `.csv` from the UI.

**Module 5 — MAVLink Integration.** A `pymavlink` listener ingests real MAVLink over UDP (HEARTBEAT, GLOBAL_POSITION_INT, VFR_HUD, SYS_STATUS, GPS_RAW_INT, RADIO_STATUS) and feeds the same dashboard. A bundled sender streams simulated MAVLink so the full protocol path can be demonstrated without hardware — and the listener works unchanged with ArduPilot/PX4 SITL or a real telemetry radio.

## Quick start

```bash
git clone <this-repo> && cd xo11-gcs/backend
pip install -r requirements.txt
python app.py
# open http://localhost:5000
```

Fly it: press **▲ TAKEOFF** → click **+ Add Waypoints** and click points on the map → **⇪ Upload Mission** → **AUTO**. The UAV flies the route, then auto-RTHs and lands. Try **HOLD / RTH / LAND** any time.

### MAVLink mode (Module 5)

Terminal 1 — start the GCS listening for MAVLink:
```bash
GCS_SOURCE=mavlink python app.py
```
Terminal 2 — stream simulated MAVLink telemetry:
```bash
python mavlink_sim_sender.py
```
Or point a real source at it: `sim_vehicle.py --out=udp:127.0.0.1:14550` (ArduPilot SITL) or a telemetry radio.

### Static demo deploy (Vercel / Netlify)

Deploy the `frontend/` folder as a static site. The dashboard detects there's no backend and automatically switches to a built-in **browser demo simulator** (badge shows "SOURCE: BROWSER DEMO") — all five UI modules stay fully interactive, with battery drain accelerated so the alert cascade is visible in a short demo video.

### Run tests

```bash
cd backend && python3 test_offline.py    # 37 checks, no dependencies needed
```

## Architecture

```
frontend/index.html  ── SSE /stream ──►  Flask (app.py)
   Leaflet map                             ├─ TelemetryHub (1 producer → N subscribers)
   Chart.js graphs                         ├─ AlertEngine (hysteresis rules)
   Demo-mode fallback sim                  ├─ MissionLog (txt/csv export)
                                           └─ source:
                                               ├─ simulator.py (default)
                                               └─ mavlink_listener.py ◄─UDP─ mavlink_sim_sender.py / SITL / real UAV
```

Key decisions:

- **SSE over WebSockets** — telemetry is strictly server→client; SSE gives auto-reconnect for free, works through proxies, and needs zero extra dependencies. Commands go the other way as plain REST POSTs.
- **Source-agnostic telemetry shape** — the simulator and the MAVLink listener emit the identical dict, so the frontend doesn't know or care where data comes from. Swapping in a real UAV is a config change, not a rewrite.
- **Calibrated physics, not random numbers** — 6S Li-ion pack with a real discharge curve (4.20→3.30 V/cell) and load-dependent voltage sag; power draw varies with climb/cruise/hover/headwind; ground speed = airspeed + wind component; RF signal follows log-distance path loss from the home antenna; turn rate and climb rate are limited like a real airframe. Flight-time-remaining is computed from live current draw with a 10% reserve.
- **Safety-first command handling** — takeoff refused below 20% battery, AUTO refused with no mission, waypoint altitudes validated to a 5–400 m envelope, and a battery-critical failsafe forces RTH regardless of operator input.

## API

| Method | Path | Purpose |
|---|---|---|
| GET | `/stream` | SSE telemetry + alerts + log tail (4 Hz) |
| GET | `/api/status` | Source + latest snapshot |
| POST | `/api/command/{takeoff\|auto\|manual\|hold\|rth\|land}` | Flight commands |
| POST | `/api/waypoints` | Upload mission `{"waypoints":[{"lat","lon","alt"}]}` |
| DELETE | `/api/waypoints` | Clear mission |
| GET | `/api/log` | Full mission log (JSON) |
| GET | `/api/log/export?format=txt\|csv` | Download log |

## Project structure

```
xo11-gcs/
├── backend/
│   ├── app.py                 # Flask server, SSE hub, alert engine, mission log
│   ├── simulator.py           # calibrated flight/battery/GPS/RF physics
│   ├── mavlink_listener.py    # Module 5: pymavlink UDP ingest
│   ├── mavlink_sim_sender.py  # Module 5: simulated MAVLink stream
│   ├── test_offline.py        # dependency-free test suite
│   └── requirements.txt
└── frontend/
    └── index.html             # entire dashboard (Leaflet + Chart.js, single file)
```

---
*XO11 UAV Systems — Real Project Assignment, July 2026*

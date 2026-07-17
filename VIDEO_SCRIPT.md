# XO11 GCS — Demo Video Shooting Script

Target length: 3–5 minutes. Record at 1920×1080, browser full-screen (hide bookmarks bar), system audio off, mic on. Do one silent rehearsal first — the flight timing below assumes you know where each button is.

## Setup (before recording)

```bash
cd xo11-gcs/backend
GCS_BATT_ACCEL=30 python app.py
```

`GCS_BATT_ACCEL=30` makes the battery story happen inside your video instead of over 70 minutes. Open http://localhost:5000, press ⟲ RESET so you start clean at 100%, switch the map to **Dark ops** layer, and zoom so the geofence ring fills most of the screen.

## Scene-by-scene

**0:00 — Cold open on the dashboard (20 s).**
"This is XO11's Ground Control Station — the software that flies our UAV. Python and Flask on the backend, vanilla JavaScript with Leaflet and Chart.js on the front, telemetry streamed at 4 hertz over Server-Sent Events." Mouse over the telemetry panel, the mode annunciator, the geofence ring.

**0:20 — Mission planning (Module 1) (40 s).**
Click **+ Add Waypoints**, place 3–4 waypoints spread around inside the fence (put the last one close to the fence edge — you'll use it later). Drag one waypoint to show editing, right-click delete one, then **⇪ Upload Mission**. Say: "Waypoints are validated server-side — altitude envelope, geofence, coordinate sanity. Watch the mission log confirm the upload."

**1:00 — Takeoff and live telemetry (Module 2) (40 s).**
Press **▲ TAKEOFF**, then **AUTO** once it reaches altitude. While it flies the route: point at altitude/airspeed/ground-speed ("they differ — that's simulated wind"), battery voltage sagging under load, the flight-time estimate, satellites, the compass. Show the breadcrumb trail and the charts trending at the bottom.

**1:40 — FPV view (30 s).**
Click **⌖ FPV View**. Let a waypoint turn happen on camera: "The OSD horizon banks with real coordinated-turn physics, the compass tape tracks heading, and the terrain is the same free satellite imagery — no API keys anywhere in this project."

**2:10 — Alerts and failsafes (Module 3) — the centerpiece (60 s).**
The battery is now draining visibly. Narrate the cascade as it happens: LOW alert at 25% (amber banner, logged, audible tone), CRITICAL at 15% — "and the GCS doesn't ask the operator, it acts: automatic Return-To-Home." Point at the mode annunciator flipping to RTH in amber. Mention hysteresis: "alerts don't flap at thresholds — they trigger at 25 but only clear at 27, and GPS alerts need three sustained seconds."

**3:10 — Mission log + export (Module 4) (20 s).**
Scroll the log: arm, mode changes, waypoints reached, alerts, failsafe. Click **Export .csv**, show the downloaded file for two seconds.

**3:30 — MAVLink (Module 5) (30 s).**
Split terminal on screen. Kill the server, run:
```bash
GCS_SOURCE=mavlink python app.py      # terminal 1
python mavlink_sim_sender.py          # terminal 2
```
Refresh the browser: "Same dashboard, but now every number is arriving as real MAVLink packets over UDP — heartbeat, position, VFR HUD, system status — parsed by pymavlink. Point this at ArduPilot SITL or a telemetry radio and nothing changes."

**4:00 — Close (20 s).**
Show the airframe selector (switch Scout → Heavy, note voltage jump 16.8 V → 50.4 V), then one line: "Sixty-plus automated tests, hardened against corrupt telemetry, link loss, and operator error. Repo, report, and live demo linked below." End on the dashboard mid-flight.

## Tips

- If an alert fires while you're mid-sentence, roll with it — reacting to live alerts looks better than scripted silence.
- Keep the cursor calm; move it only to what you're talking about.
- If a take goes wrong, press ⟲ RESET and restart the flight — no server restart needed.
- Record the MAVLink scene separately and cut it in if juggling terminals live is awkward.
- For the deployed-link bonus: show the Vercel URL in demo mode for 10 seconds ("no backend here — the dashboard detects that and runs its built-in simulator").

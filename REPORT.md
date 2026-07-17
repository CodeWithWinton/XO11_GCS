# XO11 Ground Control Station — Project Report

**Project:** Ground Control Station (GCS) Software — Real Product Development
**Team:** CS / IT · XO11 UAV Systems
**Stack:** Python (Flask, pymavlink) · Vanilla JavaScript · Leaflet.js · Chart.js
**Delivery:** July 2026

---

## 1. Architecture Decisions

### 1.1 Server-Sent Events over REST polling and WebSockets

The defining constraint of a ground station is its data shape: telemetry flows almost entirely in one direction, from vehicle to operator, at a steady cadence. Commands flow the other way, but they are rare and discrete — a mode change, a mission upload. Designing around that asymmetry led to the central transport decision of this project.

REST polling was rejected first. Polling at 4 Hz means the browser opens, negotiates, and tears down four HTTP requests every second to ask a question the server already knows the answer to. WebSockets solve that, but bring cost of their own: an upgrade handshake, a bidirectional framing protocol we would use in only one direction, an extra server dependency, and hand-rolled reconnection logic on the client.

Server-Sent Events sit exactly on the problem. A single long-lived HTTP response becomes a lightweight one-way pipeline: the backend writes a JSON frame whenever the producer ticks, and the browser's native `EventSource` delivers it — with automatic reconnection built into the browser itself. The server side is a plain generator on a Flask route; there is no additional dependency anywhere in the system. Commands travel over ordinary REST POSTs (`/api/command/rth`, `/api/waypoints`), which keeps the control path explicit, individually loggable, and trivially testable with curl. At 4 Hz with a handful of subscribers, the fan-out is handled by a publish/subscribe hub with per-client bounded queues, so one slow browser tab can never back-pressure the telemetry loop.

### 1.2 A source-agnostic producer pipeline

The backend is organized around a single producer loop that owns time. On each tick it pulls a snapshot from a *telemetry source*, runs the alert engine over it, appends any events to the mission log, and publishes one consolidated frame to all SSE subscribers.

The critical property is that the two sources — the internal physics simulator and the raw UDP MAVLink listener (via pymavlink) — emit the **identical dictionary shape**. The MAVLink listener ingests `HEARTBEAT`, `GLOBAL_POSITION_INT`, `VFR_HUD`, `SYS_STATUS`, `GPS_RAW_INT`, and `RADIO_STATUS` packets and standardizes them into the same fields the simulator produces (converting units on the way: m/s to km/h, centidegrees to degrees, millivolts to volts). Downstream of that seam, nothing knows or cares where the data came from. Swapping from simulation to a real vehicle is an environment variable, not a rewrite — which is precisely the property a real product needs when hardware arrives after the software.

The simulator behind the default source is a calibrated quadcopter model rather than a random-number generator: a 6S Li-ion pack with a real discharge curve and load-dependent voltage sag computed from internal resistance, power draw that varies across climb, cruise, and hover with a headwind penalty, and a log-distance path-loss RF model for link quality. This mattered for testing: realistic dynamics are what exposed several of the bugs described in Section 2.

### 1.3 The Demo Mode fallback

A dashboard that requires a running Python backend cannot be hosted on a static platform like Vercel — yet a live demo link was a submission requirement. Rather than maintain two frontends, the client detects backend absence at startup: if the `EventSource` errors before its first message ever arrives, the UI seamlessly boots an in-browser replica of the physics simulator and drives the entire interface from it. Every module — map, telemetry, alerts, mission log, failsafes — remains fully interactive, with the source badge honestly reporting "BROWSER DEMO." The same detection logic doubles as resilience: if a *previously connected* backend drops, the client instead enters exponential-backoff reconnection, because falling back to fake data on a live operation would be dangerous rather than helpful.

---

## 2. Challenges Faced

### 2.1 Alert flapping at thresholds

The naive alert implementation compared a value against a threshold every tick. In practice, a battery hovering at exactly 25% produced a stream of raise/clear/raise transitions — an alarm that cries wolf four times a second trains the operator to ignore it. The fix was **hysteresis**: each rule has separate trigger and clear thresholds (battery LOW trips below 25% but does not clear until 27%; GPS trips below 6 satellites but clears at 7). Fast-fluctuating signals needed a second mechanism on top: **persistence debouncing**, where GPS and RF conditions must hold continuously for three seconds before the alert raises at all. A one-second satellite blip now never reaches the operator; a genuine degradation episode does, exactly once, with a timestamp.

### 2.2 Corrupt and missing MAVLink telemetry

MAVLink encodes "unknown" as in-band sentinels: battery percentage arrives as `-1`, voltage as `65535` mV, RSSI as `255`. Fed raw into the pipeline, these produced two distinct failures — a false *Battery CRITICAL* alarm (−1% is, after all, below 15%), and in one code path an unguarded comparison against `None` that threw a `TypeError` inside the telemetry loop, silently killing it while the UI continued showing stale data as "LINK OK." The repair was end-to-end: sentinels are mapped to `null` at the listener boundary, every alert rule reads values through a numeric-safe accessor, the UI renders "—" rather than a misleading zero, and every producer tick is exception-guarded so a single malformed packet can degrade one frame but never the system. A frontend stale-data watchdog closes the last gap: if frames stop arriving on an open socket, the header flips to "LINK STALE" within six seconds.

### 2.3 Wind drift in position-hold states

Repeated automated mission runs (the simulator is stochastic, so the test suite runs full flights) exposed a physics error: the UAV would land up to a hundred meters from home. The model computed ground speed as airspeed plus wind component in *all* states — so during HOLD and LAND, with zero commanded airspeed, the aircraft drifted downwind like a balloon. A real multirotor's flight controller cancels wind drift when position-holding. The fix injects velocity-cancellation for stationary flight states: when commanded speed is zero and airspeed has decayed, ground speed is clamped to zero, while forward flight retains the full wind interaction.

### 2.4 Safety boundaries

Because this software will sit in front of a real aircraft, operator error and vehicle state had to be bounded, not trusted. The final system enforces a **1500 m geofence** (rendered on the map; breach raises a critical alert and forces an automatic Return-To-Home, with waypoints outside the fence rejected at upload), a **battery-critical auto-RTH** at 15%, a **battery-exhaustion forced landing** at 0%, refusal to arm below 20% charge, a mission-feasibility check that compares route length plus the return leg against estimated range at current battery, and strict input validation on every API surface (altitude envelope, coordinate range and finiteness, waypoint count).

---

## 3. What I Learned

**The MAVLink protocol, from bytes to dashboard.** Working with pymavlink demystified how real autopilots communicate: heartbeat-based liveness, the split of state across message types (`VFR_HUD` for the pilot's instruments, `SYS_STATUS` for power, `GPS_RAW_INT` for fix quality), custom-mode enums that differ per firmware, and the pervasive convention of in-band sentinel values. The translation layer from ArduPilot's native units into operator-facing numbers is where most ground-station bugs live.

**Bridging low-level streams into web UIs.** This project connected a UDP packet stream to a 60 fps browser interface through a chain of well-defined seams — listener, normalizer, producer loop, SSE hub, render loop. The lesson that generalizes: pick the transport that matches the data's actual shape, and normalize at the boundary so everything downstream stays simple.

**UAV physics is a systems problem.** Battery behavior is not a linearly draining percentage — voltage sags under load, climb draws nearly twice the power of hover, and a sustained headwind measurably shortens flight time. Modeling this honestly changed the *software* design: flight-time-remaining needed an exponential moving average to be readable, and alerting needed hysteresis because real signals are noisy.

**Hardening is a discipline, not a feature.** The difference between a demo and a product turned out to be a long list of small, specific defenses: HTML-escaping everything that touches `innerHTML`, watchdogs for silent failure modes, debounced alerts, exception-guarded loops, reconnecting listeners, bounded queues and capped logs, and failsafes that respect operator intent while refusing to let the vehicle be lost. Each one was cheap; finding the *need* for each one required deliberately hunting edge cases rather than waiting for them.

---

*XO11 UAV Systems — Real Project Assignment, July 2026*

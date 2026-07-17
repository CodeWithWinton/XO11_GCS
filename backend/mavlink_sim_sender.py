"""
XO11 GCS — Simulated MAVLink Sender (Module 5 companion)

Streams realistic MAVLink telemetry over UDP so the GCS can be tested
end-to-end through the real protocol path without a physical UAV.

Run:
    python mavlink_sim_sender.py               # sends to 127.0.0.1:14550
    python mavlink_sim_sender.py --hz 4

Then start the GCS with:
    GCS_SOURCE=mavlink python app.py
"""

import argparse
import time

from pymavlink import mavutil

from simulator import UAVSimulator


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--target", default="127.0.0.1:14550")
    ap.add_argument("--hz", type=float, default=4.0)
    args = ap.parse_args()

    conn = mavutil.mavlink_connection(f"udpout:{args.target}",
                                      source_system=1, source_component=1)
    mav = conn.mav

    sim = UAVSimulator(tick_hz=args.hz)
    sim.arm_and_takeoff()
    # small demo mission around home
    sim.set_waypoints([
        {"lat": sim.home_lat + 0.004, "lon": sim.home_lon + 0.002, "alt": 60},
        {"lat": sim.home_lat + 0.003, "lon": sim.home_lon + 0.006, "alt": 80},
        {"lat": sim.home_lat - 0.002, "lon": sim.home_lon + 0.004, "alt": 60},
    ])

    mode_to_custom = {"MANUAL": 0, "AUTO": 3, "HOLD": 5, "RTH": 6,
                      "LAND": 9, "TAKEOFF": 3, "IDLE": 0}

    print(f"[mavlink-sim] streaming to udpout:{args.target} at {args.hz} Hz")
    period = 1.0 / args.hz
    boot = time.time()
    try:
        while True:
            s = sim.step()
            ms = int((time.time() - boot) * 1000)

            mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_QUADROTOR,
                mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                (mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                 | (mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED if s["armed"] else 0)),
                mode_to_custom.get(s["mode"], 0),
                mavutil.mavlink.MAV_STATE_ACTIVE,
            )
            mav.global_position_int_send(
                ms,
                int(s["lat"] * 1e7), int(s["lon"] * 1e7),
                int(s["alt"] * 1000), int(s["alt"] * 1000),
                int(s["groundspeed"] / 3.6 * 100), 0,
                int(-s["climb"] * 100),
                int(s["heading"] * 100),
            )
            mav.vfr_hud_send(
                s["airspeed"] / 3.6, s["groundspeed"] / 3.6,
                int(s["heading"]), 50, s["alt"], s["climb"],
            )
            mav.sys_status_send(
                0, 0, 0, 500,
                int(s["battery_voltage"] * 1000),
                int(s["current_a"] * 100),
                int(s["battery_pct"]),
                0, 0, 0, 0, 0, 0,
            )
            mav.gps_raw_int_send(
                ms * 1000, 3,
                int(s["lat"] * 1e7), int(s["lon"] * 1e7),
                int(s["alt"] * 1000),
                int(s["hdop"] * 100), 65535,
                int(s["groundspeed"] / 3.6 * 100), 65535,
                s["sats"],
            )
            time.sleep(period)
    except KeyboardInterrupt:
        print("\n[mavlink-sim] stopped")


if __name__ == "__main__":
    main()

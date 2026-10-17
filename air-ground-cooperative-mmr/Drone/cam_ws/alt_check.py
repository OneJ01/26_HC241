#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import math
import threading
import time
from pathlib import Path

from pymavlink import mavutil


# =========================
# 설정
# =========================
CONNECTION_STRING = "/dev/ttyACM0"   # 예: /dev/ttyACM0, udpin:0.0.0.0:14550
BAUDRATE = 115200

PRINT_HZ = 2.0
SAVE_CSV = True
CSV_PATH = Path.home() / "alt_check_log.csv"

MESSAGE_RATES = {
    "GLOBAL_POSITION_INT": 5.0,
    "GPS_RAW_INT": 2.0,
    "VFR_HUD": 2.0,
    "HOME_POSITION": 1.0,
    "AHRS2": 2.0,
    "LOCAL_POSITION_NED": 2.0,
    "TERRAIN_REPORT": 2.0,
    "RANGEFINDER": 2.0,
    "DISTANCE_SENSOR": 2.0,
}

MSG_ID = {
    "GPS_RAW_INT": 24,
    "LOCAL_POSITION_NED": 32,
    "GLOBAL_POSITION_INT": 33,
    "VFR_HUD": 74,
    "DISTANCE_SENSOR": 132,
    "TERRAIN_REPORT": 136,
    "RANGEFINDER": 173,
    "AHRS2": 178,
    "HOME_POSITION": 242,
}


# =========================
# 상태 저장
# =========================
state_lock = threading.Lock()
stop_event = threading.Event()

telemetry_state = {
    "is_connected": False,

    "position_abs_alt_m": None,
    "position_rel_alt_m": None,

    "raw_abs_alt_m": None,
    "raw_ellipsoid_alt_m": None,

    "vfr_hud_alt_m": None,
    "home_alt_msl_m": None,
    "ahrs2_altitude_m": None,

    "local_z_down_m": None,

    "terrain_height_msl_m": None,
    "terrain_current_height_m": None,

    "rangefinder_distance_m": None,
    "distance_sensor_current_distance_m": None,

    "error_text": "",
}


# =========================
# 유틸
# =========================
def fmt(value, digits=3):
    if value is None:
        return "N/A"
    if isinstance(value, float):
        if math.isnan(value):
            return "N/A"
        return f"{value:.{digits}f}"
    return str(value)


def mm_to_m(value):
    if value is None:
        return None
    return value / 1000.0


def cm_to_m(value):
    if value is None:
        return None
    return value / 100.0


def safe_getattr(obj, name, default=None):
    try:
        return getattr(obj, name, default)
    except Exception:
        return default


def update_state(**kwargs):
    with state_lock:
        telemetry_state.update(kwargs)


def get_snapshot():
    with state_lock:
        return dict(telemetry_state)


def clear_terminal():
    print("\033[2J\033[H", end="")


def init_csv(path: Path):
    if path.exists():
        return

    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([
            "pc_time",
            "position_abs_alt_m",
            "position_rel_alt_m",
            "raw_abs_alt_m",
            "raw_ellipsoid_alt_m",
            "vfr_hud_alt_m",
            "home_alt_msl_m",
            "ahrs2_altitude_m",
            "local_z_down_m",
            "terrain_height_msl_m",
            "terrain_current_height_m",
            "rangefinder_distance_m",
            "distance_sensor_current_distance_m",
        ])


def append_csv(path: Path, snapshot: dict):
    with open(path, "a", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([
            time.strftime("%Y-%m-%d %H:%M:%S"),
            snapshot["position_abs_alt_m"],
            snapshot["position_rel_alt_m"],
            snapshot["raw_abs_alt_m"],
            snapshot["raw_ellipsoid_alt_m"],
            snapshot["vfr_hud_alt_m"],
            snapshot["home_alt_msl_m"],
            snapshot["ahrs2_altitude_m"],
            snapshot["local_z_down_m"],
            snapshot["terrain_height_msl_m"],
            snapshot["terrain_current_height_m"],
            snapshot["rangefinder_distance_m"],
            snapshot["distance_sensor_current_distance_m"],
        ])


# =========================
# 메시지 요청
# =========================
def request_message_interval(master, msg_id, hz):
    if hz <= 0:
        return

    interval_us = int(1_000_000 / hz)
    try:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            interval_us,
            0, 0, 0, 0, 0
        )
    except Exception:
        pass


def request_home_position(master):
    try:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
    except Exception:
        pass


def request_all_intervals(master):
    try:
        master.mav.request_data_stream_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            2,
            1
        )
    except Exception:
        pass

    for name, hz in MESSAGE_RATES.items():
        request_message_interval(master, MSG_ID[name], hz)
        time.sleep(0.03)

    request_home_position(master)


# =========================
# 메시지 처리
# =========================
def handle_global_position_int(msg):
    update_state(
        position_abs_alt_m=mm_to_m(safe_getattr(msg, "alt", None)),
        position_rel_alt_m=mm_to_m(safe_getattr(msg, "relative_alt", None)),
    )


def handle_gps_raw_int(msg):
    update_state(
        raw_abs_alt_m=mm_to_m(safe_getattr(msg, "alt", None)),
        raw_ellipsoid_alt_m=mm_to_m(safe_getattr(msg, "alt_ellipsoid", None)),
    )


def handle_vfr_hud(msg):
    update_state(
        vfr_hud_alt_m=safe_getattr(msg, "alt", None),
    )


def handle_home_position(msg):
    update_state(
        home_alt_msl_m=mm_to_m(safe_getattr(msg, "altitude", None)),
    )


def handle_ahrs2(msg):
    update_state(
        ahrs2_altitude_m=safe_getattr(msg, "altitude", None),
    )


def handle_local_position_ned(msg):
    update_state(
        local_z_down_m=safe_getattr(msg, "z", None),
    )


def handle_terrain_report(msg):
    update_state(
        terrain_height_msl_m=safe_getattr(msg, "terrain_height", None),
        terrain_current_height_m=safe_getattr(msg, "current_height", None),
    )


def handle_rangefinder(msg):
    update_state(
        rangefinder_distance_m=safe_getattr(msg, "distance", None),
    )


def handle_distance_sensor(msg):
    update_state(
        distance_sensor_current_distance_m=cm_to_m(safe_getattr(msg, "current_distance", None)),
    )


def handle_message(msg):
    mtype = msg.get_type()

    if mtype == "BAD_DATA":
        return
    elif mtype == "GLOBAL_POSITION_INT":
        handle_global_position_int(msg)
    elif mtype == "GPS_RAW_INT":
        handle_gps_raw_int(msg)
    elif mtype == "VFR_HUD":
        handle_vfr_hud(msg)
    elif mtype == "HOME_POSITION":
        handle_home_position(msg)
    elif mtype == "AHRS2":
        handle_ahrs2(msg)
    elif mtype == "LOCAL_POSITION_NED":
        handle_local_position_ned(msg)
    elif mtype == "TERRAIN_REPORT":
        handle_terrain_report(msg)
    elif mtype == "RANGEFINDER":
        handle_rangefinder(msg)
    elif mtype == "DISTANCE_SENSOR":
        handle_distance_sensor(msg)


# =========================
# MAVLink 수신 스레드
# =========================
def mavlink_worker():
    try:
        print(f"[INFO] Connecting to {CONNECTION_STRING}")

        if CONNECTION_STRING.startswith("/dev/"):
            master = mavutil.mavlink_connection(
                CONNECTION_STRING,
                baud=BAUDRATE,
                autoreconnect=True,
                source_system=255,
            )
        else:
            master = mavutil.mavlink_connection(
                CONNECTION_STRING,
                autoreconnect=True,
                source_system=255,
            )

        master.wait_heartbeat(timeout=30)
        print("[INFO] Heartbeat received")
        print("[INFO] Connected")
        print("[INFO] Press Ctrl+C to stop")

        update_state(is_connected=True)
        request_all_intervals(master)

        while not stop_event.is_set():
            msg = master.recv_match(blocking=True, timeout=1.0)
            if msg is None:
                continue
            handle_message(msg)

    except Exception as e:
        update_state(error_text=str(e))
        print(f"[ERR] {e}")


# =========================
# 화면 출력
# =========================
def build_screen(snapshot):
    lines = []
    lines.append("================================ ALT CHECK ================================")
    lines.append("")
    lines.append(f"GLOBAL_POSITION_INT.alt            : {fmt(snapshot['position_abs_alt_m'], 6)} m")
    lines.append(f"GLOBAL_POSITION_INT.relative_alt   : {fmt(snapshot['position_rel_alt_m'], 6)} m")
    lines.append("")
    lines.append(f"GPS_RAW_INT.alt                    : {fmt(snapshot['raw_abs_alt_m'], 6)} m")
    lines.append(f"GPS_RAW_INT.alt_ellipsoid          : {fmt(snapshot['raw_ellipsoid_alt_m'], 6)} m")
    lines.append("")
    lines.append(f"VFR_HUD.alt                        : {fmt(snapshot['vfr_hud_alt_m'], 6)} m")
    lines.append(f"HOME_POSITION.altitude             : {fmt(snapshot['home_alt_msl_m'], 6)} m")
    lines.append(f"AHRS2.altitude                     : {fmt(snapshot['ahrs2_altitude_m'], 6)} m")
    lines.append("")
    lines.append(f"LOCAL_POSITION_NED.z               : {fmt(snapshot['local_z_down_m'], 6)} m")
    lines.append("")
    lines.append(f"TERRAIN_REPORT.terrain_height      : {fmt(snapshot['terrain_height_msl_m'], 6)} m")
    lines.append(f"TERRAIN_REPORT.current_height      : {fmt(snapshot['terrain_current_height_m'], 6)} m")
    lines.append("")
    lines.append(f"RANGEFINDER.distance               : {fmt(snapshot['rangefinder_distance_m'], 6)} m")
    lines.append(f"DISTANCE_SENSOR.current_distance   : {fmt(snapshot['distance_sensor_current_distance_m'], 6)} m")
    lines.append("")
    if snapshot["error_text"]:
        lines.append(f"ERROR                              : {snapshot['error_text']}")
        lines.append("")
    lines.append(f"CSV                                : {CSV_PATH}")
    lines.append("=========================================================================")
    return "\n".join(lines)


def print_loop():
    period = 1.0 / PRINT_HZ

    while not stop_event.is_set():
        snapshot = get_snapshot()
        clear_terminal()
        print(build_screen(snapshot))

        if SAVE_CSV:
            append_csv(CSV_PATH, snapshot)

        time.sleep(period)


# =========================
# 메인
# =========================
def main():
    if SAVE_CSV:
        init_csv(CSV_PATH)

    t = threading.Thread(target=mavlink_worker, daemon=True)
    t.start()

    try:
        print_loop()
    except KeyboardInterrupt:
        stop_event.set()
        print("\n[INFO] Stopped by user")


if __name__ == "__main__":
    main()
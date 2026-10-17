#!/usr/bin/env python3
import asyncio
import json
import time
import os
import math
import hashlib
import shutil
import threading
import re
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MissionManager(Node):
    """
    Final-style FC manager for Raspberry Pi Docker environment.

    역할:
    - FC serial/MAVSDK 연결을 이 노드 하나가 단독 소유한다.
    - /drone/telemetry 발행
    - /mission/upload_request 수신
    - Mission Planner .waypoints 파일 동기화
    - MAVSDK MissionRaw clear/upload/read-back
    - /mission/path, /mission/status 발행
    - mission_progress 기반 RUNNING/COMPLETED/current_wp 발행

    실행 예:
        cd /root/cam_ws/scripts
        export GCS_WS=/root/cam_ws
        export FC_SYSTEM_ADDRESS=serial:///dev/ttyACM0:115200
        python3 mission_manager.py
    """

    def __init__(self):
        super().__init__("mission_manager")

        base_ws = os.path.expanduser(os.environ.get("GCS_WS", "~/h264_gcs_ws"))
        self.mission_dir = os.path.expanduser(
            os.environ.get("MISSION_DIR", os.path.join(base_ws, "missions"))
        )
        os.makedirs(self.mission_dir, exist_ok=True)

        self.fc_system_address = os.environ.get(
            "FC_SYSTEM_ADDRESS",
            "serial:///dev/ttyACM0:115200",
        )
        self.fc_connect_timeout_s = float(os.environ.get("FC_CONNECT_TIMEOUT_S", "20.0"))
        self.telemetry_pub_hz = float(os.environ.get("TELEMETRY_PUB_HZ", "10.0"))

        self.pub_status = self.create_publisher(String, "/mission/status", 10)
        self.pub_path = self.create_publisher(String, "/mission/path", 10)
        self.pub_telemetry = self.create_publisher(String, "/drone/telemetry", 10)

        self.sub_upload = self.create_subscription(
            String,
            "/mission/upload_request",
            self.on_upload_request,
            10,
        )

        self._busy = False

        # FC/MAVSDK 단일 연결 관련
        self.drone = None
        self._fc_loop = None
        self._fc_thread = None
        self._fc_stop_event = threading.Event()
        self._fc_ready_event = threading.Event()
        self._fc_connected = False
        self._fc_error = None

        # Telemetry state
        self._telemetry_lock = threading.Lock()
        self._telemetry_state = {
            "armed": None,
            "flight_mode": None,
            "in_air": None,
            "landed_state": None,
            "roll_deg": None,
            "pitch_deg": None,
            "yaw_deg": None,
            "lat_deg": None,
            "lon_deg": None,
            "abs_alt_m": None,
            "rel_alt_m": None,
            "vel_n_m_s": None,
            "vel_e_m_s": None,
            "vel_d_m_s": None,
            "ground_speed_m_s": None,
            "gps_fix_type": None,
            "num_satellites": None,
            "battery_v": None,
            "battery_remaining_pct": None,
            "health": None,
            "status_text": None,
            "telem_time_ns": None,
            "fc_connected": False,
            "stream_errors": {},
        }

        # Mission progress state
        self._progress_seq_offset = 0
        self._active_mission_id = None
        self._active_total_wp = None
        self._last_progress_key = None
        self._mission_was_running = False
        self._mission_completed = False
        self._mission_effective_state = "IDLE"
        self._last_paused_key = None

        self._start_fc_thread()

        self.get_logger().info("Mission/FC manager started")
        self.get_logger().info(f"mission_dir: {self.mission_dir}")
        self.get_logger().info(f"fc_system_address: {self.fc_system_address}")
        self.get_logger().info("FC serial owner: this process only")

    # ------------------------------------------------------------
    # ROS publish helpers
    # ------------------------------------------------------------
    def publish_status(self, mission_id, state, current_wp=None, total_wp=None, message="", **extra):
        payload = {
            "mission_id": mission_id,
            "state": state,
            "current_wp": current_wp,
            "total_wp": total_wp,
            "message": message,
            "stamp": time.time(),
        }
        if extra:
            payload.update(extra)

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub_status.publish(msg)

    def publish_path_points(self, mission_id, points, current_wp=None, **extra):
        payload = {
            "mission_id": mission_id,
            "points": points,
            "current_wp": current_wp,
            "total_wp": len(points),
            "stamp": time.time(),
        }
        if extra:
            payload.update(extra)

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub_path.publish(msg)

    def _publish_telemetry(self):
        with self._telemetry_lock:
            payload = dict(self._telemetry_state)
            payload["telem_time_ns"] = time.time_ns()

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.pub_telemetry.publish(msg)

    # ------------------------------------------------------------
    # File / hash helpers
    # ------------------------------------------------------------
    def _mission_file_path(self, mission_id: str) -> str:
        mission_id = str(mission_id).strip().upper()
        return os.path.join(self.mission_dir, f"mission_{mission_id}.waypoints")

    def _sha256_text(self, text: str) -> str:
        return hashlib.sha256(text.encode("utf-8")).hexdigest()

    def _sha256_file(self, path: str) -> str | None:
        if not os.path.exists(path):
            return None
        h = hashlib.sha256()
        with open(path, "rb") as f:
            for chunk in iter(lambda: f.read(65536), b""):
                h.update(chunk)
        return h.hexdigest()

    def _save_or_update_mission_file(self, mission_id: str, filename: str, text: str, incoming_sha256: str):
        mission_file = self._mission_file_path(mission_id)

        calc_sha256 = self._sha256_text(text)
        if calc_sha256 != incoming_sha256:
            raise ValueError(
                f"sha256 mismatch: received={incoming_sha256[:12]}..., calculated={calc_sha256[:12]}..."
            )

        old_sha256 = self._sha256_file(mission_file)
        if old_sha256 == incoming_sha256:
            return mission_file, "same"

        if os.path.exists(mission_file):
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            backup_file = mission_file + f".bak_{ts}"
            shutil.copy2(mission_file, backup_file)
            self.get_logger().info(f"backup old mission file: {backup_file}")
            mode = "updated"
        else:
            mode = "created"

        with open(mission_file, "w", encoding="utf-8", newline="\n") as f:
            f.write(text)

        return mission_file, mode

    # ------------------------------------------------------------
    # Mission Planner / QGC WPL parse helpers
    # ------------------------------------------------------------
    def parse_waypoints_text(self, text: str):
        cmd_map = {
            16: "WAYPOINT",
            17: "LOITER_UNLIM",
            18: "LOITER_TURNS",
            19: "LOITER_TIME",
            20: "RTL",
            21: "LAND",
            22: "TAKEOFF",
        }

        lines = text.splitlines()
        if not lines:
            raise RuntimeError("empty mission text")
        if not lines[0].strip().startswith("QGC WPL"):
            raise RuntimeError("not QGC WPL format")

        points = []
        home_lat = None
        home_lon = None
        home_alt = None
        last_valid_lat = None
        last_valid_lon = None
        last_valid_alt = None

        for line in lines[1:]:
            line = line.strip()
            if not line:
                continue
            cols = line.split()
            if len(cols) < 11:
                continue

            try:
                seq = int(cols[0])
                current_flag = int(cols[1])
                frame = int(float(cols[2]))
                cmd_num = int(float(cols[3]))
                lat = float(cols[8])
                lon = float(cols[9])
                alt = float(cols[10])
            except Exception:
                continue

            # Mission Planner/QGC WPL의 실제 HOME 행만 제외한다.
            # seq=0/current=1만으로 판단하면 HOME 제거 후 seq=0이 된 TAKEOFF를
            # HOME으로 오인할 수 있으므로 frame/command까지 함께 확인한다.
            is_home_row = (
                seq == 0
                and current_flag == 1
                and frame == 0
                and cmd_num in (0, 16)
            )
            if is_home_row:
                if abs(lat) > 1e-9 or abs(lon) > 1e-9:
                    home_lat = lat
                    home_lon = lon
                    home_alt = alt
                    self.get_logger().info(
                        f"store HOME row: lat={home_lat}, lon={home_lon}, alt={home_alt}"
                    )
                continue

            cmd = cmd_map.get(cmd_num, f"CMD_{cmd_num}")

            if abs(lat) < 1e-9 and abs(lon) < 1e-9:
                cmd_upper = cmd.upper()
                if "TAKEOFF" in cmd_upper and home_lat is not None and home_lon is not None:
                    lat = home_lat
                    lon = home_lon
                    if abs(alt) < 1e-9 and home_alt is not None:
                        alt = home_alt
                elif last_valid_lat is not None and last_valid_lon is not None:
                    lat = last_valid_lat
                    lon = last_valid_lon
                    if abs(alt) < 1e-9 and last_valid_alt is not None:
                        alt = last_valid_alt

            if abs(lat) < 1e-9 and abs(lon) < 1e-9:
                self.get_logger().warn(f"skip seq {seq}: invalid lat/lon 0,0")
                continue

            points.append({
                "seq": seq,
                "lat": lat,
                "lon": lon,
                "alt": alt,
                "cmd": cmd,
                "cmd_num": cmd_num,
            })

            last_valid_lat = lat
            last_valid_lon = lon
            last_valid_alt = alt

        if not points:
            raise RuntimeError("no valid mission points parsed")
        return points

    def _filter_imported_duplicate_home(self, mission_items):
        """
        MAVSDK Mission Planner importer는 ArduPilot용 HOME을 seq 0에 자동 추가한다.
        입력 WPL에도 seq 0 HOME 행이 있으면 다음과 같이 중복될 수 있다.

            raw 0: MAVSDK가 자동 추가한 ArduPilot HOME
            raw 1: WPL 원본의 HOME 행
            raw 2: TAKEOFF

        이 함수는 첫 번째 자동 HOME은 유지하고, 그 뒤의 frame=0/CMD0·16
        원본 HOME 행만 제거한 뒤 seq/current를 다시 정리한다.
        """
        items = list(mission_items)
        if not items:
            raise RuntimeError("MAVSDK import returned no mission_items")

        filtered = []
        removed_rows = []

        for idx, item in enumerate(items):
            try:
                frame = int(getattr(item, "frame", 0))
                cmd_num = int(getattr(item, "command", 0))
                x_raw = int(getattr(item, "x", 0))
                y_raw = int(getattr(item, "y", 0))
                z = float(getattr(item, "z", 0.0))
            except Exception as e:
                raise RuntimeError(
                    f"failed to inspect imported mission item index={idx}: {e}"
                ) from e

            # idx 0은 MAVSDK가 ArduPilot 호환을 위해 추가한 HOME이므로 유지한다.
            is_duplicate_file_home = (
                idx > 0
                and frame == 0
                and cmd_num in (0, 16)
            )

            if is_duplicate_file_home:
                removed_rows.append({
                    "index": idx,
                    "frame": frame,
                    "cmd_num": cmd_num,
                    "lat": float(x_raw) / 1e7,
                    "lon": float(y_raw) / 1e7,
                    "alt": z,
                })
                continue

            filtered.append(item)

        if not filtered:
            raise RuntimeError("no mission items after duplicate HOME filtering")

        # MissionRaw upload 조건에 맞게 seq/current를 연속적으로 다시 정리한다.
        for new_seq, item in enumerate(filtered):
            item.seq = new_seq
            item.current = 1 if new_seq == 0 else 0

        executable_items = filtered[1:] if len(filtered) > 1 else []
        if not executable_items:
            raise RuntimeError("no executable mission items after HOME filtering")

        first_exec_cmd = int(getattr(executable_items[0], "command", 0))
        if first_exec_cmd != 22:
            raise RuntimeError(
                "first executable mission item is not TAKEOFF after HOME filtering: "
                f"command={first_exec_cmd}"
            )

        info = {
            "imported_item_count": len(items),
            "removed_duplicate_home_count": len(removed_rows),
            "removed_duplicate_home_rows": removed_rows,
            "upload_item_count": len(filtered),
            "expected_executable_count": len(executable_items),
            "first_executable_command": first_exec_cmd,
        }

        if removed_rows:
            for row in removed_rows:
                self.get_logger().info(
                    "duplicate WPL HOME removed before FC upload: "
                    f"index={row['index']} frame={row['frame']} cmd={row['cmd_num']} "
                    f"lat={row['lat']:.7f} lon={row['lon']:.7f} alt={row['alt']:.3f}"
                )
        else:
            self.get_logger().warn(
                "duplicate WPL HOME row not found in MAVSDK import result; "
                "mission may already be sanitized or importer behavior may differ"
            )

        self.get_logger().info(
            "mission import filter result: "
            f"imported={len(items)}, upload={len(filtered)}, "
            f"executable={len(executable_items)}, "
            f"removed_home={len(removed_rows)}"
        )

        return filtered, info

    def _raw_items_debug_entries(self, mission_items):
        """
        FC에서 download_mission()으로 다시 받은 MissionRaw item을 UI 보정 없이 그대로 기록한다.
        핵심: x/y가 0이면 0 그대로 남긴다. HOME/TAKEOFF/LAND 좌표 보정을 하지 않는다.
        """
        cmd_map = {
            16: "WAYPOINT",
            17: "LOITER_UNLIM",
            18: "LOITER_TURNS",
            19: "LOITER_TIME",
            20: "RTL",
            21: "LAND",
            22: "TAKEOFF",
        }

        rows = []
        for idx, item in enumerate(mission_items):
            try:
                raw_seq = int(getattr(item, "seq", idx))
                frame = int(getattr(item, "frame", 0))
                cmd_num = int(getattr(item, "command", 0))
                current = int(getattr(item, "current", 0))
                autocontinue = int(getattr(item, "autocontinue", 0))
                x_raw = int(getattr(item, "x", 0))
                y_raw = int(getattr(item, "y", 0))
                z = float(getattr(item, "z", 0.0))
                param1 = float(getattr(item, "param1", 0.0))
                param2 = float(getattr(item, "param2", 0.0))
                param3 = float(getattr(item, "param3", 0.0))
                param4 = float(getattr(item, "param4", 0.0))
            except Exception as e:
                rows.append({"index": idx, "parse_error": str(e)})
                continue

            rows.append({
                "index": idx,
                "raw_seq": raw_seq,
                "frame": frame,
                "cmd": cmd_map.get(cmd_num, f"CMD_{cmd_num}"),
                "cmd_num": cmd_num,
                "current": current,
                "autocontinue": autocontinue,
                "param1": param1,
                "param2": param2,
                "param3": param3,
                "param4": param4,
                "x_raw": x_raw,
                "y_raw": y_raw,
                "z": z,
                "lat_raw": float(x_raw) / 1e7,
                "lon_raw": float(y_raw) / 1e7,
            })
        return rows

    def _is_raw_home_or_dummy(self, item):
        try:
            frame = int(getattr(item, "frame", 0))
            cmd_num = int(getattr(item, "command", 0))
        except Exception:
            return False

        try:
            raw_seq = int(getattr(item, "seq", 0))
        except Exception:
            raw_seq = 0

        # ArduPilot HOME은 read-back에서 frame 0 또는 GLOBAL_INT(5)로 보일 수 있다.
        # 실제 경유 WP는 일반적으로 frame 3이므로 구분된다.
        return (
            cmd_num in (0, 16)
            and (
                frame == 0
                or (raw_seq == 0 and frame == 5)
            )
        )

    def _first_executable_raw_seq(self, mission_items):
        for idx, item in enumerate(mission_items):
            if self._is_raw_home_or_dummy(item):
                continue
            try:
                return int(getattr(item, "seq", idx))
            except Exception:
                return idx
        return 0

    def _raw_seq_to_ui_seq(self, points, raw_seq):
        if raw_seq is None:
            return None
        try:
            raw_i = int(raw_seq)
        except Exception:
            return None
        for p in points or []:
            try:
                if int(p.get("raw_seq", -999999)) == raw_i:
                    return int(p.get("seq"))
            except Exception:
                continue
        return None

    async def _reset_current_mission_to_first_async(self, mission_items):
        """
        업로드/read-back 완료 직후 AUTO 진입 시 시작할 raw mission item을 명시적으로 첫 실행 항목으로 맞춘다.
        목적: 이전 AUTO 중단 지점이나 이전 mission current pointer 잔재를 차단한다.
        """
        first_seq = self._first_executable_raw_seq(mission_items)
        info = {
            "reset_requested": True,
            "reset_done": False,
            "fc_current_raw": first_seq,
            "reset_error": None,
        }

        mission_raw = getattr(self.drone, "mission_raw", None)
        setter = getattr(mission_raw, "set_current_mission_item", None)
        if setter is None:
            info["reset_requested"] = False
            info["reset_error"] = "MAVSDK mission_raw.set_current_mission_item not available"
            self.get_logger().warn(info["reset_error"])
            return info

        try:
            await setter(first_seq)
            info["reset_done"] = True
            self.get_logger().info(f"FC current mission item reset OK: raw_seq={first_seq}")
        except Exception as e:
            info["reset_error"] = f"{type(e).__name__}: {e}"
            self.get_logger().error(f"FC current mission item reset failed: raw_seq={first_seq}, {e}")

        return info

    def _raw_items_to_ui_points(self, mission_items):
        cmd_map = {
            16: "WAYPOINT",
            17: "LOITER_UNLIM",
            18: "LOITER_TURNS",
            19: "LOITER_TIME",
            20: "RTL",
            21: "LAND",
            22: "TAKEOFF",
        }

        raw_entries = []
        home_lat = None
        home_lon = None
        home_alt = None
        last_valid_lat = None
        last_valid_lon = None
        last_valid_alt = None

        for item in mission_items:
            try:
                raw_seq = int(getattr(item, "seq", len(raw_entries)))
                frame = int(getattr(item, "frame", 0))
                cmd_num = int(getattr(item, "command", 0))
                x_raw = int(getattr(item, "x", 0))
                y_raw = int(getattr(item, "y", 0))
                alt = float(getattr(item, "z", 0.0))
            except Exception:
                continue

            cmd = cmd_map.get(cmd_num, f"CMD_{cmd_num}")
            lat = float(x_raw) / 1e7
            lon = float(y_raw) / 1e7

            # MAVSDK/ArduPilot HOME/더미 row는 지도 경로 항목에서 제외한다.
            if self._is_raw_home_or_dummy(item):
                if abs(lat) > 1e-9 or abs(lon) > 1e-9:
                    home_lat = lat
                    home_lon = lon
                    home_alt = alt
                    self.get_logger().info(
                        f"read-back HOME row stored: seq={raw_seq}, lat={home_lat}, lon={home_lon}, alt={home_alt}"
                    )
                else:
                    self.get_logger().info(f"read-back dummy row skipped: seq={raw_seq}")
                continue

            if abs(lat) < 1e-9 and abs(lon) < 1e-9:
                cmd_upper = cmd.upper()
                if "TAKEOFF" in cmd_upper and home_lat is not None and home_lon is not None:
                    lat = home_lat
                    lon = home_lon
                    if abs(alt) < 1e-9 and home_alt is not None:
                        alt = home_alt
                elif last_valid_lat is not None and last_valid_lon is not None:
                    lat = last_valid_lat
                    lon = last_valid_lon
                    if abs(alt) < 1e-9 and last_valid_alt is not None:
                        alt = last_valid_alt

            if abs(lat) < 1e-9 and abs(lon) < 1e-9:
                self.get_logger().warn(
                    f"read-back skip seq {raw_seq}: invalid lat/lon 0,0, cmd={cmd}"
                )
                continue

            raw_entries.append({
                "raw_seq": raw_seq,
                "lat": lat,
                "lon": lon,
                "alt": alt,
                "cmd": cmd,
                "cmd_num": cmd_num,
            })

            last_valid_lat = lat
            last_valid_lon = lon
            last_valid_alt = alt

        if not raw_entries:
            raise RuntimeError("no valid mission points from FC read-back")

        first_raw_seq = int(raw_entries[0]["raw_seq"])
        seq_offset = max(0, first_raw_seq - 1)
        self._progress_seq_offset = seq_offset

        points = []
        for idx, e in enumerate(raw_entries):
            raw_seq = int(e["raw_seq"])
            ui_seq = raw_seq - seq_offset
            if ui_seq <= 0:
                ui_seq = idx + 1

            points.append({
                "seq": ui_seq,
                "lat": e["lat"],
                "lon": e["lon"],
                "alt": e["alt"],
                "cmd": e["cmd"],
                "cmd_num": e["cmd_num"],
                "raw_seq": raw_seq,
                "source": "fc_readback",
            })

        self.get_logger().info(
            f"FC read-back converted to UI points={len(points)}, seq_offset={seq_offset}"
        )
        return points

    # ------------------------------------------------------------
    # FC single connection thread
    # ------------------------------------------------------------
    def _start_fc_thread(self):
        self._fc_thread = threading.Thread(target=self._fc_thread_main, daemon=True)
        self._fc_thread.start()

    def _fc_thread_main(self):
        self._fc_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._fc_loop)
        try:
            self._fc_loop.run_until_complete(self._fc_main())
        except Exception as e:
            self._fc_error = str(e)
            self.get_logger().error(f"FC loop failed: {type(e).__name__}: {e}")
        finally:
            try:
                pending = asyncio.all_tasks(self._fc_loop)
                for task in pending:
                    task.cancel()
                if pending:
                    self._fc_loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
            except Exception:
                pass
            try:
                self._fc_loop.close()
            except Exception:
                pass

    async def _fc_main(self):
        from mavsdk import System

        self.drone = System()
        self.get_logger().info(f"FC connecting once: {self.fc_system_address}")
        await self.drone.connect(system_address=self.fc_system_address)
        await self._wait_connected(self.drone)

        self._fc_connected = True
        self._fc_ready_event.set()
        self._set_telemetry_value("fc_connected", True)
        self.get_logger().info("FC connected once; telemetry/mission share this MAVSDK connection")

        await asyncio.sleep(2.0)
        await self._request_telemetry_rates(self.drone)
        await asyncio.sleep(1.0)

        tasks = [
            asyncio.create_task(self._safe_runner("armed", self._watch_armed(self.drone))),
            asyncio.create_task(self._safe_runner("flight_mode", self._watch_flight_mode(self.drone))),
            asyncio.create_task(self._safe_runner("in_air", self._watch_in_air(self.drone))),
            asyncio.create_task(self._safe_runner("landed_state", self._watch_landed_state(self.drone))),
            asyncio.create_task(self._safe_runner("attitude_euler", self._watch_euler_angle(self.drone))),
            asyncio.create_task(self._safe_runner("position", self._watch_position(self.drone))),
            asyncio.create_task(self._safe_runner("velocity_ned", self._watch_velocity_ned(self.drone))),
            asyncio.create_task(self._safe_runner("gps_info", self._watch_gps_info(self.drone))),
            asyncio.create_task(self._safe_runner("battery", self._watch_battery(self.drone))),
            asyncio.create_task(self._safe_runner("health", self._watch_health(self.drone))),
            asyncio.create_task(self._safe_runner("status_text", self._watch_status_text(self.drone))),
            asyncio.create_task(self._safe_runner("mission_progress", self._watch_mission_progress(self.drone))),
            asyncio.create_task(self._safe_runner("telemetry_publish", self._telemetry_publish_loop())),
            asyncio.create_task(self._safe_runner("pause_state", self._pause_state_loop())),
        ]

        try:
            while not self._fc_stop_event.is_set():
                await asyncio.sleep(0.2)
        finally:
            for task in tasks:
                task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)

    async def _wait_connected(self, drone):
        async def _inner():
            async for state in drone.core.connection_state():
                if state.is_connected:
                    return
        await asyncio.wait_for(_inner(), timeout=self.fc_connect_timeout_s)

    def _wait_fc_ready_sync(self, timeout_s=20.0):
        if not self._fc_ready_event.wait(timeout=timeout_s):
            raise RuntimeError(f"FC not connected yet: {self._fc_error or self.fc_system_address}")
        if self.drone is None or self._fc_loop is None:
            raise RuntimeError("FC loop not ready")

    async def _safe_runner(self, name, coro):
        try:
            await coro
        except asyncio.CancelledError:
            raise
        except Exception as e:
            with self._telemetry_lock:
                self._telemetry_state.setdefault("stream_errors", {})[name] = str(e)
            self.get_logger().warn(f"{name} stream stopped: {e}")

    def _set_telemetry_value(self, key, value):
        with self._telemetry_lock:
            self._telemetry_state[key] = value
            self._telemetry_state["telem_time_ns"] = time.time_ns()

    def _update_telemetry_values(self, values: dict):
        with self._telemetry_lock:
            self._telemetry_state.update(values)
            self._telemetry_state["telem_time_ns"] = time.time_ns()

    async def _request_telemetry_rates(self, drone):
        """
        MAVSDK Python 버전마다 지원하는 set_rate_* 함수가 다를 수 있으므로,
        존재하는 함수만 호출한다. 특히 일부 버전에는 set_rate_status_text가 없다.
        """
        rate_specs = [
            ("attitude_euler", "set_rate_attitude_euler", 10.0),
            ("position", "set_rate_position", 10.0),
            ("velocity_ned", "set_rate_velocity_ned", 10.0),
            ("gps_info", "set_rate_gps_info", 2.0),
            ("battery", "set_rate_battery", 2.0),
            ("status_text", "set_rate_status_text", 2.0),
        ]

        for name, method_name, hz in rate_specs:
            fn = getattr(drone.telemetry, method_name, None)
            if fn is None:
                self.get_logger().warn(f"set_rate_{name} skipped: {method_name} not supported")
                continue

            try:
                await fn(hz)
                self.get_logger().info(f"set_rate_{name}({hz}) OK")
            except Exception as e:
                self.get_logger().warn(f"set_rate_{name} failed: {e}")

    # ------------------------------------------------------------
    # Telemetry watchers
    # ------------------------------------------------------------
    async def _watch_armed(self, drone):
        async for armed in drone.telemetry.armed():
            self._set_telemetry_value("armed", bool(armed))

    async def _watch_flight_mode(self, drone):
        last_mode_text = None

        async for flight_mode in drone.telemetry.flight_mode():
            mode_text = self._normalize_flight_mode(flight_mode)
            self._set_telemetry_value("flight_mode", mode_text)

            if mode_text != last_mode_text:
                last_mode_text = mode_text
                self.get_logger().info(
                    f"flight_mode changed: raw={repr(flight_mode)}, "
                    f"str={str(flight_mode)}, normalized={mode_text}"
                )

    async def _watch_in_air(self, drone):
        async for in_air in drone.telemetry.in_air():
            self._set_telemetry_value("in_air", bool(in_air))

    async def _watch_landed_state(self, drone):
        async for landed_state in drone.telemetry.landed_state():
            self._set_telemetry_value("landed_state", str(landed_state))

    async def _watch_euler_angle(self, drone):
        async for euler in drone.telemetry.attitude_euler():
            self._update_telemetry_values({
                "roll_deg": euler.roll_deg,
                "pitch_deg": euler.pitch_deg,
                "yaw_deg": euler.yaw_deg,
            })

    async def _watch_position(self, drone):
        async for position in drone.telemetry.position():
            self._update_telemetry_values({
                "lat_deg": position.latitude_deg,
                "lon_deg": position.longitude_deg,
                "abs_alt_m": position.absolute_altitude_m,
                "rel_alt_m": position.relative_altitude_m,
            })

    async def _watch_velocity_ned(self, drone):
        async for vel in drone.telemetry.velocity_ned():
            gs = None
            try:
                gs = math.sqrt(vel.north_m_s * vel.north_m_s + vel.east_m_s * vel.east_m_s)
            except Exception:
                pass
            self._update_telemetry_values({
                "vel_n_m_s": vel.north_m_s,
                "vel_e_m_s": vel.east_m_s,
                "vel_d_m_s": vel.down_m_s,
                "ground_speed_m_s": gs,
            })

    async def _watch_gps_info(self, drone):
        async for gps in drone.telemetry.gps_info():
            self._update_telemetry_values({
                "gps_fix_type": str(gps.fix_type),
                "num_satellites": gps.num_satellites,
            })

    async def _watch_battery(self, drone):
        async for battery in drone.telemetry.battery():
            batt_pct = battery.remaining_percent
            if batt_pct is not None and batt_pct < 0.0:
                batt_pct = None
            self._update_telemetry_values({
                "battery_v": battery.voltage_v,
                "battery_remaining_pct": batt_pct,
            })

    async def _watch_health(self, drone):
        async for health in drone.telemetry.health():
            self._set_telemetry_value("health", {
                "is_gyrometer_calibration_ok": getattr(health, "is_gyrometer_calibration_ok", None),
                "is_accelerometer_calibration_ok": getattr(health, "is_accelerometer_calibration_ok", None),
                "is_magnetometer_calibration_ok": getattr(health, "is_magnetometer_calibration_ok", None),
                "is_local_position_ok": getattr(health, "is_local_position_ok", None),
                "is_global_position_ok": getattr(health, "is_global_position_ok", None),
                "is_home_position_ok": getattr(health, "is_home_position_ok", None),
                "is_armable": getattr(health, "is_armable", None),
            })

    async def _watch_status_text(self, drone):
        async for status_text in drone.telemetry.status_text():
            self._set_telemetry_value("status_text", f"[{status_text.type}] {status_text.text}")

    async def _telemetry_publish_loop(self):
        period = 1.0 / max(0.1, self.telemetry_pub_hz)
        while not self._fc_stop_event.is_set():
            try:
                self._publish_telemetry()
            except Exception as e:
                self.get_logger().warn(f"telemetry publish failed: {e}")
            await asyncio.sleep(period)

    # ------------------------------------------------------------
    # Mission progress / PAUSED / COMPLETED / ABORTED
    # ------------------------------------------------------------
    def _normalize_flight_mode(self, mode):
        """
        MAVSDK/ArduPilot flight mode 값을 비교 가능한 문자열로 정리.
        예:
        - FlightMode.MISSION -> MISSION
        - Mission            -> MISSION
        - Auto               -> AUTO
        - GUIDED             -> GUIDED
        """
        if mode is None:
            return ""

        name = getattr(mode, "name", None)
        if name:
            s = str(name).upper()
        else:
            s = str(mode).upper()

        s = s.replace("FLIGHTMODE.", "")
        s = s.replace("FLIGHT_MODE.", "")
        s = s.replace(" ", "_")
        return s.strip()

    def _mode_tokens(self, mode):
        s = self._normalize_flight_mode(mode)
        return s, set(re.split(r"[^A-Z0-9]+", s))

    def _is_mission_mode(self, mode):
        """
        자동 미션 주행으로 인정할 mode만 True.

        ArduPilot Copter 실제 미션 모드명: AUTO
        MAVSDK 추상 표현 가능성: MISSION

        GUIDED는 GCS/companion이 동적으로 유도하는 모드라서
        Mission Planner .waypoints 자동미션 주행으로 보지 않음.
        """
        s, tokens = self._mode_tokens(mode)

        if not s:
            return False

        if "GUIDED" in tokens:
            return False

        return ("AUTO" in tokens) or ("MISSION" in tokens)

    def _is_terminal_auto_mode(self, mode):
        """
        LAND/RTL 계열은 최종 착륙/복귀 과정에서 잠깐 보일 수 있으므로,
        마지막 WP 근처에서는 PAUSED 오판 방지용으로 사용.
        """
        s, tokens = self._mode_tokens(mode)

        return (
            "LAND" in tokens
            or "RTL" in tokens
            or "RETURN" in tokens
            or "RETURN_TO_LAUNCH" in s
        )

    def _get_last_current_wp(self):
        if self._last_progress_key is None:
            return None
        try:
            return int(self._last_progress_key[1])
        except Exception:
            return None

    async def _watch_mission_progress(self, drone):
        async for progress in drone.mission_raw.mission_progress():
            if self._fc_stop_event.is_set():
                break

            mission_id = self._active_mission_id
            total_wp = self._active_total_wp
            if mission_id is None or total_wp is None:
                continue

            try:
                raw_current = int(getattr(progress, "current", 0))
                raw_total = int(getattr(progress, "total", 0))
            except Exception:
                continue

            seq_offset = int(getattr(self, "_progress_seq_offset", 0))
            if raw_current <= seq_offset:
                continue

            ui_current = raw_current - seq_offset
            if ui_current <= 0:
                ui_current = 1
            if ui_current > total_wp:
                ui_current = total_wp

            finished = False
            try:
                finished = bool(await drone.mission_raw.is_mission_finished())
            except Exception:
                finished = False

            if finished:
                state = "COMPLETED"
            elif self._mission_effective_state == "PAUSED":
                state = "PAUSED"
            else:
                state = "RUNNING"

            key = (state, ui_current, total_wp)

            if key != self._last_progress_key:
                self._last_progress_key = key
                self._mission_was_running = True
                self._mission_completed = finished
                self._mission_effective_state = state

                if finished:
                    msg = f"Mission {mission_id} 완료"
                elif state == "PAUSED":
                    msg = f"Mission {mission_id} 일시정지 유지: SEQ {ui_current}"
                else:
                    msg = f"Mission {mission_id} 진행 중: SEQ {ui_current}"

                self.publish_status(
                    mission_id,
                    state,
                    current_wp=ui_current,
                    total_wp=total_wp,
                    message=msg,
                )
                self.get_logger().info(
                    f"mission progress: raw={raw_current}/{raw_total}, "
                    f"ui={ui_current}/{total_wp}, state={state}"
                )

            if finished:
                break

    async def _pause_state_loop(self):
        while not self._fc_stop_event.is_set():
            await asyncio.sleep(0.5)

            mission_id = self._active_mission_id
            total_wp = self._active_total_wp
            if mission_id is None or total_wp is None:
                continue

            if not self._mission_was_running:
                continue

            if self._mission_effective_state in ("COMPLETED", "ABORTED"):
                continue

            with self._telemetry_lock:
                mode = self._telemetry_state.get("flight_mode")
                armed = self._telemetry_state.get("armed")

            current_wp = self._get_last_current_wp()

            # 1) RUNNING/PAUSED 중 DISARM 감지
            if armed is False:
                if self._mission_completed:
                    final_state = "COMPLETED"
                    final_msg = f"Mission {mission_id} 완료 후 DISARM"
                else:
                    final_state = "ABORTED"
                    final_msg = f"Mission {mission_id} 중단 후 DISARM"

                if self._mission_effective_state != final_state:
                    self._mission_effective_state = final_state
                    self._mission_was_running = False

                    self.publish_status(
                        mission_id,
                        final_state,
                        current_wp=current_wp,
                        total_wp=total_wp,
                        message=final_msg,
                    )
                    self.get_logger().warn(
                        f"mission {final_state.lower()}: "
                        f"mode={mode}, current_wp={current_wp}"
                    )
                continue

            if armed is not True:
                continue

            if mode is None:
                continue

            mission_mode = self._is_mission_mode(mode)
            terminal_mode = self._is_terminal_auto_mode(mode)

            near_end = current_wp is not None and current_wp >= total_wp

            # 2) AUTO/MISSION 이탈 감지
            if not mission_mode:
                # LAND/RTL 최종 구간 오판 방지
                if terminal_mode and near_end:
                    continue

                if self._mission_effective_state == "RUNNING":
                    self._mission_effective_state = "PAUSED"

                    key = ("PAUSED", current_wp, str(mode))
                    if key != self._last_paused_key:
                        self._last_paused_key = key

                        self.publish_status(
                            mission_id,
                            "PAUSED",
                            current_wp=current_wp,
                            total_wp=total_wp,
                            message=f"AUTO/MISSION 이탈 감지: mode={mode}",
                        )
                        self.get_logger().warn(
                            f"mission paused: mode={mode}, current_wp={current_wp}"
                        )

            # 3) PAUSED 상태에서 AUTO/MISSION 복귀 감지
            else:
                if self._mission_effective_state == "PAUSED":
                    self._mission_effective_state = "RUNNING"
                    self._last_paused_key = None

                    self.publish_status(
                        mission_id,
                        "RUNNING",
                        current_wp=current_wp,
                        total_wp=total_wp,
                        message=f"AUTO/MISSION 복귀 감지: mode={mode}",
                    )
                    self.get_logger().info(
                        f"mission resumed: mode={mode}, current_wp={current_wp}"
                    )

    # ------------------------------------------------------------
    # Mission upload using the single FC connection
    # ------------------------------------------------------------
    async def _read_armed_once(self, timeout_s=3.0):
        async def _inner():
            async for armed in self.drone.telemetry.armed():
                return bool(armed)
        try:
            return await asyncio.wait_for(_inner(), timeout=timeout_s)
        except Exception:
            with self._telemetry_lock:
                armed = self._telemetry_state.get("armed")
            return armed

    async def _upload_to_fc_and_readback_async(self, mission_id: str, mission_text: str, total_wp: int):
        if self.drone is None:
            raise RuntimeError("FC drone object is not ready")

        self.publish_status(
            mission_id,
            "CONNECTING",
            current_wp=None,
            total_wp=total_wp,
            message=f"FC 연결 확인 중: {self.fc_system_address}",
        )

        # background thread에서 이미 연결을 유지하므로 여기서는 연결 완료만 확인
        if not self._fc_connected:
            await self._wait_connected(self.drone)
            self._fc_connected = True
            self._fc_ready_event.set()

        armed = await self._read_armed_once()
        if armed is True:
            raise RuntimeError("FC is ARMED; mission upload blocked for safety")
        elif armed is None:
            self.get_logger().warn("armed state unknown; continuing mission upload")
        else:
            self.get_logger().info("FC armed=False")

        self.publish_status(
            mission_id,
            "UPLOADING",
            current_wp=None,
            total_wp=total_wp,
            message="Mission Planner 파일을 MAVSDK MissionRaw로 변환 중",
        )

        import_data = await self.drone.mission_raw.import_mission_planner_mission_from_string(
            mission_text
        )
        imported_items = list(import_data.mission_items)
        mission_items, upload_info = self._filter_imported_duplicate_home(imported_items)

        self.publish_status(
            mission_id,
            "CLEARING",
            current_wp=None,
            total_wp=total_wp,
            message="FC 기존 미션 삭제 중",
        )
        await self.drone.mission_raw.clear_mission()
        self.get_logger().info("FC mission cleared")

        self.publish_status(
            mission_id,
            "UPLOADING",
            current_wp=None,
            total_wp=total_wp,
            message=f"FC 미션 업로드 중: {len(mission_items)} items",
        )
        await self.drone.mission_raw.upload_mission(mission_items)
        self.get_logger().info("FC mission upload OK")

        self.publish_status(
            mission_id,
            "VERIFYING",
            current_wp=None,
            total_wp=total_wp,
            message="FC 미션 read-back 검증 중",
        )
        downloaded_items = await self.drone.mission_raw.download_mission()
        downloaded_items = list(downloaded_items)
        self.get_logger().info(f"FC mission read-back items={len(downloaded_items)}")

        if len(downloaded_items) <= 0:
            raise RuntimeError("FC read-back returned empty mission")

        raw_debug_entries = self._raw_items_debug_entries(downloaded_items)
        for e in raw_debug_entries:
            if "parse_error" in e:
                self.get_logger().warn(f"FC READBACK RAW parse error: {e}")
                continue
            self.get_logger().info(
                "FC READBACK RAW "
                f"seq={e['raw_seq']} frame={e['frame']} cmd={e['cmd']}({e['cmd_num']}) "
                f"cur={e['current']} auto={e['autocontinue']} "
                f"x={e['x_raw']} y={e['y_raw']} z={e['z']} "
                f"lat={e['lat_raw']:.7f} lon={e['lon_raw']:.7f}"
            )

        home_like_items = [
            item for item in downloaded_items
            if self._is_raw_home_or_dummy(item)
        ]
        executable_items = [
            item for item in downloaded_items
            if not self._is_raw_home_or_dummy(item)
        ]

        if len(home_like_items) > 1:
            raise RuntimeError(
                "FC read-back duplicate HOME detected: "
                f"home_rows={len(home_like_items)}. "
                "Mission Planner에서 1번 WAYPOINT 중복 여부 확인 필요"
            )

        expected_exec_count = int(upload_info.get("expected_executable_count", 0))
        if expected_exec_count and len(executable_items) != expected_exec_count:
            raise RuntimeError(
                "FC read-back executable item count mismatch: "
                f"expected={expected_exec_count}, actual={len(executable_items)}"
            )

        if executable_items:
            first_exec_cmd = int(getattr(executable_items[0], "command", 0))
            if first_exec_cmd != 22:
                raise RuntimeError(
                    "FC read-back first executable item is not TAKEOFF: "
                    f"command={first_exec_cmd}"
                )

        upload_info["fc_home_row_count"] = len(home_like_items)
        upload_info["fc_executable_item_count"] = len(executable_items)

        reset_info = await self._reset_current_mission_to_first_async(downloaded_items)

        return downloaded_items, raw_debug_entries, reset_info, upload_info

    def _run_upload_on_fc_loop(self, mission_id: str, mission_text: str, total_wp: int):
        self._wait_fc_ready_sync(timeout_s=self.fc_connect_timeout_s + 5.0)
        future = asyncio.run_coroutine_threadsafe(
            self._upload_to_fc_and_readback_async(mission_id, mission_text, total_wp),
            self._fc_loop,
        )
        return future.result(timeout=120.0)

    # ------------------------------------------------------------
    # ROS upload request callback
    # ------------------------------------------------------------
    def on_upload_request(self, msg):
        mission_id = "UNKNOWN"

        if self._busy:
            self.publish_status(
                mission_id,
                "BUSY",
                current_wp=None,
                total_wp=None,
                message="Mission manager is busy",
            )
            return

        self._busy = True

        try:
            data = json.loads(msg.data)

            mission_id = str(data.get("mission_id", "UNKNOWN")).strip().upper()
            filename = str(data.get("filename", f"mission_{mission_id}.waypoints"))
            incoming_sha256 = str(data.get("sha256", "")).strip()
            waypoints_text = data.get("waypoints_text")

            if mission_id == "UNKNOWN" or not mission_id:
                raise ValueError("mission_id missing")
            if not incoming_sha256:
                raise ValueError("sha256 missing")
            if not isinstance(waypoints_text, str) or not waypoints_text.strip():
                raise ValueError("waypoints_text missing")

            self.get_logger().info(
                f"upload request received: Mission {mission_id}, filename={filename}, sha256={incoming_sha256[:12]}..."
            )
            # FC가 ARMED 상태면 파일 저장/미션 초기화 전에 업로드를 조기 차단한다.
            # PAUSED 상태는 보통 ARMED=True이므로 여기서 자동 차단된다.
            with self._telemetry_lock:
                cached_armed = self._telemetry_state.get("armed")

            if cached_armed is True:
                self.publish_status(
                    mission_id,
                    "REJECTED",
                    current_wp=None,
                    total_wp=None,
                    message="FC ARMED 상태이므로 새 미션 업로드 차단",
                )
                self.get_logger().warn(
                    f"mission upload rejected: FC is ARMED, mission_id={mission_id}"
                )
                return
            # 새 업로드가 시작되면 기존 progress 상태 초기화
            self._active_mission_id = None
            self._active_total_wp = None
            self._last_progress_key = None
            self._last_paused_key = None
            self._mission_was_running = False
            self._mission_completed = False
            self._mission_effective_state = "SYNCING"

            preview_points = self.parse_waypoints_text(waypoints_text)
            total_wp = len(preview_points)

            self.publish_status(
                mission_id,
                "SYNCING",
                current_wp=None,
                total_wp=total_wp,
                message=f"Mission {mission_id} 파일 수신 완료, 동기화 중",
            )

            mission_file, sync_mode = self._save_or_update_mission_file(
                mission_id=mission_id,
                filename=filename,
                text=waypoints_text,
                incoming_sha256=incoming_sha256,
            )

            if sync_mode == "same":
                sync_msg = f"Mission {mission_id} 파일 이미 동기화됨"
            elif sync_mode == "created":
                sync_msg = f"Mission {mission_id} 파일 새로 저장"
            else:
                sync_msg = f"Mission {mission_id} 파일 변경 감지, 백업 후 갱신"

            self.get_logger().info(sync_msg)
            self.get_logger().info(f"mission file ready: {mission_file}")

            # PC 원본 미리보기 경로를 먼저 표시
            self.publish_path_points(mission_id, preview_points, current_wp=None)

            downloaded_items, raw_debug_entries, reset_info, upload_info = self._run_upload_on_fc_loop(
                mission_id,
                waypoints_text,
                total_wp,
            )
            readback_points = self._raw_items_to_ui_points(downloaded_items)

            fc_current_raw = reset_info.get("fc_current_raw") if isinstance(reset_info, dict) else None
            fc_current_ui = self._raw_seq_to_ui_seq(readback_points, fc_current_raw)

            # FC에 실제 저장된 read-back 경로로 UI 갱신
            self.publish_path_points(
                mission_id,
                readback_points,
                current_wp=fc_current_ui,
                source="fc_readback",
                fc_current_raw=fc_current_raw,
                fc_current_ui=fc_current_ui,
            )

            # progress 감시 활성화
            self._active_mission_id = mission_id
            self._active_total_wp = len(readback_points)
            self._last_progress_key = None
            self._last_paused_key = None
            self._mission_was_running = False
            self._mission_completed = False

            self._mission_effective_state = "READY"

            reset_msg = (
                "current item reset OK"
                if reset_info.get("reset_done")
                else f"current item reset 확인 필요: {reset_info.get('reset_error')}"
            )
            home_filter_msg = (
                "중복 HOME 제거 OK"
                if upload_info.get("removed_duplicate_home_count", 0) > 0
                else "중복 HOME 없음"
            )

            self.publish_status(
                mission_id,
                "READY",
                current_wp=fc_current_ui,
                total_wp=len(readback_points),
                message=(
                    f"{sync_msg} / {home_filter_msg} / "
                    f"FC 업로드, read-back 완료 / {reset_msg}"
                ),
                fc_current_raw=fc_current_raw,
                fc_current_ui=fc_current_ui,
                reset_info=reset_info,
                upload_info=upload_info,
                raw_items_preview=raw_debug_entries[:12],
            )

        except Exception as e:
            self.get_logger().error(f"mission upload failed: {e}")
            self.publish_status(
                mission_id,
                "ERROR",
                current_wp=None,
                total_wp=None,
                message=f"Mission {mission_id} 업로드 실패: {e}",
            )

        finally:
            self._busy = False

    # ------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------
    def shutdown_manager(self):
        self._fc_stop_event.set()
        try:
            if self._fc_thread is not None and self._fc_thread.is_alive():
                self._fc_thread.join(timeout=2.0)
        except Exception:
            pass


def main():
    rclpy.init()
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_manager()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

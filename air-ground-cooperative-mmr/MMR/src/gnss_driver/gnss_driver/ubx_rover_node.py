#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ubx_rover_node.py
────────────────────────────────────────────────────────────────────────────
큰 보드(Rover/Heading 출력 보드)의 UBX 메시지를 직접 파싱하여 ROS 2 토픽으로 발행.

입력:
  큰 보드 USB/serial 포트

발행:
  /gnss/fix_hp         sensor_msgs/NavSatFix   UBX-NAV-HPPOSLLH 기반 고정밀 위경도
  /fix                 sensor_msgs/NavSatFix   fire_solution 호환용 동일 위치
  /gnss/fix_pvt        sensor_msgs/NavSatFix   NAV-PVT 또는 HPPOSLLH 기반 상태/위치
  /gnss/heading        std_msgs/Float64        로봇 기준 heading [deg]
                                            raw GNSS heading + heading_offset_deg
  /gnss/relposned_vec  geometry_msgs/Vector3   N/E/D baseline [m]
  /gnss/rover/status   std_msgs/String         JSON 진단 상태

설계 의도:
  - 큰 보드는 외부 VRS RTCM을 직접 받지 않습니다.
  - 큰 보드는 Lite 보드가 만든 Moving Base RTCM만 받아 heading/position을 냅니다.
  - HPPOSLLH + RELPOSNED를 기준으로 cm급 위치와 heading 상태를 안정적으로 발행합니다.
"""

from __future__ import annotations

import json
import math
import struct
import threading
import time
from collections import deque
from typing import Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64, String
import serial


# =============================================================================
# UBX 유틸
# =============================================================================


def ubx_checksum(data: bytes) -> Tuple[int, int]:
    ck_a = 0
    ck_b = 0
    for b in data:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return ck_a, ck_b


def carr_name(carr: int) -> str:
    return {0: "NONE", 1: "FLOAT", 2: "FIXED"}.get(carr, f"UNKNOWN({carr})")


def fix_type_name(fix_type: int) -> str:
    return {
        0: "NO_FIX",
        1: "DR_ONLY",
        2: "2D",
        3: "3D",
        4: "GNSS_DR",
        5: "TIME_ONLY",
    }.get(fix_type, f"UNKNOWN({fix_type})")


def circular_mean_deg(values):
    if not values:
        return 0.0
    s = sum(math.sin(math.radians(v)) for v in values)
    c = sum(math.cos(math.radians(v)) for v in values)
    return math.degrees(math.atan2(s, c)) % 360.0


def circular_abs_error_deg(value, ref):
    return abs((value - ref + 180.0) % 360.0 - 180.0)


def parse_nav_hpposllh(payload: bytes):
    # UBX-NAV-HPPOSLLH, length 36
    if len(payload) < 36:
        return None

    i_tow = struct.unpack_from("<I", payload, 4)[0]
    lon = struct.unpack_from("<i", payload, 8)[0] * 1e-7
    lat = struct.unpack_from("<i", payload, 12)[0] * 1e-7
    height = struct.unpack_from("<i", payload, 16)[0] / 1000.0
    h_msl = struct.unpack_from("<i", payload, 20)[0] / 1000.0
    lon_hp = struct.unpack_from("<b", payload, 24)[0] * 1e-9
    lat_hp = struct.unpack_from("<b", payload, 25)[0] * 1e-9
    height_hp = struct.unpack_from("<b", payload, 26)[0] * 0.0001
    h_msl_hp = struct.unpack_from("<b", payload, 27)[0] * 0.0001
    h_acc = struct.unpack_from("<I", payload, 28)[0] * 0.0001
    v_acc = struct.unpack_from("<I", payload, 32)[0] * 0.0001

    return {
        "i_tow": i_tow,
        "lat": lat + lat_hp,
        "lon": lon + lon_hp,
        "height_m": height + height_hp,
        "h_msl_m": h_msl + h_msl_hp,
        "h_acc_m": h_acc,
        "v_acc_m": v_acc,
    }


def parse_nav_pvt(payload: bytes):
    if len(payload) < 92:
        return None

    fix_type = payload[20]
    flags = payload[21]
    num_sv = payload[23]
    lon = struct.unpack_from("<i", payload, 24)[0] * 1e-7
    lat = struct.unpack_from("<i", payload, 28)[0] * 1e-7
    h_msl_m = struct.unpack_from("<i", payload, 36)[0] / 1000.0
    h_acc_m = struct.unpack_from("<I", payload, 40)[0] / 1000.0
    v_acc_m = struct.unpack_from("<I", payload, 44)[0] / 1000.0

    return {
        "fix_type": fix_type,
        "fix_name": fix_type_name(fix_type),
        "gnss_ok": bool(flags & 0x01),
        "diff": bool(flags & 0x02),
        "carr": (flags >> 6) & 0x03,
        "num_sv": num_sv,
        "lat": lat,
        "lon": lon,
        "alt_m": h_msl_m,
        "h_acc_m": h_acc_m,
        "v_acc_m": v_acc_m,
    }


def parse_nav_relposned(payload: bytes):
    # UBX-NAV-RELPOSNED, length 64
    if len(payload) < 64:
        return None

    ref_station_id = struct.unpack_from("<H", payload, 2)[0]
    i_tow = struct.unpack_from("<I", payload, 4)[0]

    # cm 단위 기본값
    rel_n_cm = struct.unpack_from("<i", payload, 8)[0]
    rel_e_cm = struct.unpack_from("<i", payload, 12)[0]
    rel_d_cm = struct.unpack_from("<i", payload, 16)[0]
    rel_len_cm = struct.unpack_from("<i", payload, 20)[0]

    # 1e-5 deg
    heading_deg = (struct.unpack_from("<i", payload, 24)[0] * 1e-5) % 360.0

    # 0.1 mm 단위 HP 보정
    rel_hp_n = struct.unpack_from("<b", payload, 32)[0] * 0.0001
    rel_hp_e = struct.unpack_from("<b", payload, 33)[0] * 0.0001
    rel_hp_d = struct.unpack_from("<b", payload, 34)[0] * 0.0001
    rel_hp_len = struct.unpack_from("<b", payload, 35)[0] * 0.0001

    n_m = rel_n_cm * 0.01 + rel_hp_n
    e_m = rel_e_cm * 0.01 + rel_hp_e
    d_m = rel_d_cm * 0.01 + rel_hp_d
    length_m = rel_len_cm * 0.01 + rel_hp_len

    acc_n = struct.unpack_from("<I", payload, 36)[0] * 0.0001
    acc_e = struct.unpack_from("<I", payload, 40)[0] * 0.0001
    acc_d = struct.unpack_from("<I", payload, 44)[0] * 0.0001
    acc_len = struct.unpack_from("<I", payload, 48)[0] * 0.0001
    acc_heading = struct.unpack_from("<I", payload, 52)[0] * 1e-5

    flags = struct.unpack_from("<I", payload, 60)[0]
    gnss_fix_ok = bool(flags & (1 << 0))
    diff_soln = bool(flags & (1 << 1))
    rel_pos_valid = bool(flags & (1 << 2))
    carr = (flags >> 3) & 0x03
    is_moving = bool(flags & (1 << 5))
    ref_pos_miss = bool(flags & (1 << 6))
    ref_obs_miss = bool(flags & (1 << 7))
    heading_valid = bool(flags & (1 << 8))
    normalized = bool(flags & (1 << 9))

    return {
        "i_tow": i_tow,
        "ref_station_id": ref_station_id,
        "n_m": n_m,
        "e_m": e_m,
        "d_m": d_m,
        "length_m": length_m,
        "heading_deg": heading_deg,
        "acc_n_m": acc_n,
        "acc_e_m": acc_e,
        "acc_d_m": acc_d,
        "acc_length_m": acc_len,
        "acc_heading_deg": acc_heading,
        "flags": flags,
        "gnss_fix_ok": gnss_fix_ok,
        "diff_soln": diff_soln,
        "rel_pos_valid": rel_pos_valid,
        "carr": carr,
        "carr_name": carr_name(carr),
        "is_moving": is_moving,
        "ref_pos_miss": ref_pos_miss,
        "ref_obs_miss": ref_obs_miss,
        "heading_valid": heading_valid,
        "normalized": normalized,
    }


# =============================================================================
# ROS 2 Node
# =============================================================================


class UbxRoverNode(Node):
    def __init__(self):
        super().__init__("ubx_rover_node")

        self.declare_parameter("serial_port", "/dev/ttyACM1")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("frame_id", "gps_link")
        self.declare_parameter("status_publish_rate_hz", 2.0)
        self.declare_parameter("publish_hpposllh_as_pvt_status", True)
        self.declare_parameter("min_fix_hacc_m", 0.05)
        self.declare_parameter("heading_filter_window", 2)
        self.declare_parameter("position_filter_window", 5)

        # GNSS baseline heading과 로봇 전방 heading 사이의 장착 오프셋 [deg]
        # 로봇 heading = GNSS raw heading + heading_offset_deg
        self.declare_parameter("heading_offset_deg", 90.0)

        self.serial_port = str(self.get_parameter("serial_port").value)
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.publish_hpposllh_as_pvt_status = bool(
            self.get_parameter("publish_hpposllh_as_pvt_status").value
        )
        self.min_fix_hacc_m = float(self.get_parameter("min_fix_hacc_m").value)
        self.heading_offset_deg = float(self.get_parameter("heading_offset_deg").value)

        heading_window = max(1, int(self.get_parameter("heading_filter_window").value))
        pos_window = max(1, int(self.get_parameter("position_filter_window").value))

        self.relpos_vec_pub = self.create_publisher(Vector3, "/gnss/relposned_vec", 10)
        self.heading_pub = self.create_publisher(Float64, "/gnss/heading", 10)
        self.fix_hp_pub = self.create_publisher(NavSatFix, "/gnss/fix_hp", 10)
        self.fix_alias_pub = self.create_publisher(NavSatFix, "/fix", 10)
        self.fix_pvt_pub = self.create_publisher(NavSatFix, "/gnss/fix_pvt", 10)
        self.status_pub = self.create_publisher(String, "/gnss/rover/status", 10)

        self.stop_event = threading.Event()
        self.state_lock = threading.Lock()

        self.latest_hppos = None
        self.latest_pvt = None
        self.latest_relpos = None
        self.last_hppos_time = 0.0
        self.last_pvt_time = 0.0
        self.last_relpos_time = 0.0
        self.checksum_fail_count = 0
        self.ubx_count = {"HPPOSLLH": 0, "PVT": 0, "RELPOSNED": 0}

        self.heading_hist = deque(maxlen=heading_window)
        self.lat_hist = deque(maxlen=pos_window)
        self.lon_hist = deque(maxlen=pos_window)
        self.alt_hist = deque(maxlen=pos_window)

        try:
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.05,
            )
        except Exception as e:
            self.get_logger().error(f"[ROVER] serial open failed: {self.serial_port} - {e}")
            raise

        self.get_logger().info(
            "ubx_rover_node 시작\n"
            f"  Rover serial  : {self.serial_port} @ {self.baudrate}\n"
            f"  frame_id      : {self.frame_id}\n"
            f"  heading offset: {self.heading_offset_deg:.2f} deg\n"
            f"  heading win   : {heading_window}, pos win: {pos_window}"
        )

        self.reader_thread = threading.Thread(target=self._serial_reader_worker, daemon=True)
        self.reader_thread.start()

        rate = float(self.get_parameter("status_publish_rate_hz").value)
        self.create_timer(1.0 / max(rate, 0.1), self._publish_status)

    # ------------------------------------------------------------------
    # Serial reader
    # ------------------------------------------------------------------
    def _serial_reader_worker(self):
        buf = bytearray()
        while rclpy.ok() and not self.stop_event.is_set():
            try:
                data = self.ser.read(4096)
                if not data:
                    continue
                buf.extend(data)
                self._parse_buffer(buf)
            except Exception as e:
                self.get_logger().warn(
                    f"[ROVER] serial read error: {e}",
                    throttle_duration_sec=2.0,
                )
                time.sleep(0.2)

    def _parse_buffer(self, buf: bytearray):
        while buf:
            idx = buf.find(b"\xB5\x62")
            if idx < 0:
                if len(buf) > 4096:
                    del buf[:-1]
                return
            if idx > 0:
                del buf[:idx]
            if len(buf) < 6:
                return

            cls_id = buf[2]
            msg_id = buf[3]
            length = struct.unpack_from("<H", buf, 4)[0]
            total_len = 6 + length + 2
            if length > 4096:
                del buf[0]
                continue
            if len(buf) < total_len:
                return

            packet = bytes(buf[:total_len])
            del buf[:total_len]

            ck_a, ck_b = ubx_checksum(packet[2:6 + length])
            if ck_a != packet[-2] or ck_b != packet[-1]:
                self.checksum_fail_count += 1
                self.get_logger().warn(
                    f"[ROVER] UBX checksum fail cls=0x{cls_id:02X} id=0x{msg_id:02X}",
                    throttle_duration_sec=2.0,
                )
                continue

            payload = packet[6:6 + length]
            self._handle_ubx(cls_id, msg_id, payload)

    def _handle_ubx(self, cls_id: int, msg_id: int, payload: bytes):
        # NAV-HPPOSLLH
        if cls_id == 0x01 and msg_id == 0x14:
            msg = parse_nav_hpposllh(payload)
            if msg:
                self._handle_hpposllh(msg)

        # NAV-PVT
        elif cls_id == 0x01 and msg_id == 0x07:
            msg = parse_nav_pvt(payload)
            if msg:
                self._handle_pvt(msg)

        # NAV-RELPOSNED
        elif cls_id == 0x01 and msg_id == 0x3C:
            msg = parse_nav_relposned(payload)
            if msg:
                self._handle_relposned(msg)

    # ------------------------------------------------------------------
    # Message handlers
    # ------------------------------------------------------------------
    def _fix_status_from_state(self, h_acc_m: float) -> Tuple[int, int]:
        with self.state_lock:
            rel = self.latest_relpos
            pvt = self.latest_pvt

        # RELPOSNED가 FIXED이고 HPPOSLLH hAcc도 cm급이면 RTK_FIX로 판정
        if (
            rel
            and rel.get("carr") == 2
            and rel.get("heading_valid")
            and h_acc_m <= self.min_fix_hacc_m
        ):
            return NavSatStatus.STATUS_GBAS_FIX, NavSatFix.COVARIANCE_TYPE_KNOWN

        # PVT가 있으면 PVT carrSoln을 보조 판정에 사용
        if pvt:
            if pvt.get("carr") == 2:
                return NavSatStatus.STATUS_GBAS_FIX, NavSatFix.COVARIANCE_TYPE_KNOWN
            if pvt.get("carr") == 1:
                return NavSatStatus.STATUS_GBAS_FIX, NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            if pvt.get("fix_type", 0) >= 2:
                return NavSatStatus.STATUS_FIX, NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        if h_acc_m <= self.min_fix_hacc_m:
            return NavSatStatus.STATUS_GBAS_FIX, NavSatFix.COVARIANCE_TYPE_KNOWN
        return NavSatStatus.STATUS_FIX, NavSatFix.COVARIANCE_TYPE_APPROXIMATED

    def _make_fix_msg(
        self,
        lat: float,
        lon: float,
        alt: float,
        h_acc: float,
        v_acc: float,
    ) -> NavSatFix:
        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = self.frame_id
        fix.latitude = lat
        fix.longitude = lon
        fix.altitude = alt
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.status.status, fix.position_covariance_type = self._fix_status_from_state(h_acc)
        fix.position_covariance = [
            h_acc * h_acc, 0.0, 0.0,
            0.0, h_acc * h_acc, 0.0,
            0.0, 0.0, v_acc * v_acc,
        ]
        return fix

    def _handle_hpposllh(self, data):
        now = time.monotonic()

        self.lat_hist.append(data["lat"])
        self.lon_hist.append(data["lon"])
        self.alt_hist.append(data["h_msl_m"])

        # 아주 작은 시리얼/계산 jitter를 줄이기 위한 짧은 중앙값 필터
        lat = sorted(self.lat_hist)[len(self.lat_hist) // 2]
        lon = sorted(self.lon_hist)[len(self.lon_hist) // 2]
        alt = sorted(self.alt_hist)[len(self.alt_hist) // 2]

        filtered = dict(data)
        filtered.update(
            {
                "lat_filtered": lat,
                "lon_filtered": lon,
                "alt_filtered_m": alt,
            }
        )

        with self.state_lock:
            self.latest_hppos = filtered
            self.last_hppos_time = now
            self.ubx_count["HPPOSLLH"] += 1

        fix_msg = self._make_fix_msg(
            lat,
            lon,
            alt,
            data["h_acc_m"],
            data["v_acc_m"],
        )
        self.fix_hp_pub.publish(fix_msg)
        self.fix_alias_pub.publish(fix_msg)

        if self.publish_hpposllh_as_pvt_status:
            self.fix_pvt_pub.publish(fix_msg)

    def _handle_pvt(self, data):
        with self.state_lock:
            self.latest_pvt = data
            self.last_pvt_time = time.monotonic()
            self.ubx_count["PVT"] += 1

        fix = self._make_fix_msg(
            data["lat"],
            data["lon"],
            data["alt_m"],
            data["h_acc_m"],
            data["v_acc_m"],
        )

        if data["carr"] == 2:
            fix.status.status = NavSatStatus.STATUS_GBAS_FIX
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_KNOWN
        elif data["carr"] == 1:
            fix.status.status = NavSatStatus.STATUS_GBAS_FIX
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        elif data["fix_type"] >= 2:
            fix.status.status = NavSatStatus.STATUS_FIX
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        else:
            fix.status.status = NavSatStatus.STATUS_NO_FIX
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self.fix_pvt_pub.publish(fix)

    def _handle_relposned(self, data):
        now = time.monotonic()

        # UBX-NAV-RELPOSNED의 heading은 두 안테나 baseline 방향입니다.
        # 로봇 전방이 baseline보다 +90° 돌아간 장착 구조이면,
        # 기존 토픽 구조는 그대로 두고 /gnss/heading 값만 로봇 기준 heading으로 보정합니다.
        raw_heading_deg = data["heading_deg"]
        corrected_heading_deg = (raw_heading_deg + self.heading_offset_deg) % 360.0

        # 기존 GUI/fire_solution 연결을 유지하기 위해 heading_deg는 보정값으로 사용합니다.
        # 디버깅을 위해 raw_heading_deg와 corrected_heading_deg는 status JSON에 함께 남깁니다.
        data = dict(data)
        data["raw_heading_deg"] = raw_heading_deg
        data["heading_offset_deg"] = self.heading_offset_deg
        data["corrected_heading_deg"] = corrected_heading_deg
        data["heading_deg"] = corrected_heading_deg

        with self.state_lock:
            self.latest_relpos = data
            self.last_relpos_time = now
            self.ubx_count["RELPOSNED"] += 1

        self.relpos_vec_pub.publish(
            Vector3(
                x=data["n_m"],
                y=data["e_m"],
                z=data["d_m"],
            )
        )

        if data["heading_valid"] and data["rel_pos_valid"] and data["carr"] == 2:
            self.heading_hist.append(data["heading_deg"])
            heading = circular_mean_deg(self.heading_hist)
            self.heading_pub.publish(Float64(data=heading))

        self.get_logger().info(
            f"[RELPOSNED] robot_hdg={data['heading_deg']:.2f}° "
            f"raw={data['raw_heading_deg']:.2f}° "
            f"offset={data['heading_offset_deg']:.2f}° "
            f"len={data['length_m']:.3f}m carr={data['carr_name']} "
            f"valid={data['rel_pos_valid']} hvalid={data['heading_valid']} "
            f"accH={data['acc_heading_deg']:.2f}°",
            throttle_duration_sec=1.0,
        )

    # ------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------
    def _publish_status(self):
        now = time.monotonic()

        with self.state_lock:
            hp = dict(self.latest_hppos) if self.latest_hppos else None
            pvt = dict(self.latest_pvt) if self.latest_pvt else None
            rel = dict(self.latest_relpos) if self.latest_relpos else None
            hp_age = now - self.last_hppos_time if self.last_hppos_time > 0 else -1.0
            pvt_age = now - self.last_pvt_time if self.last_pvt_time > 0 else -1.0
            rel_age = now - self.last_relpos_time if self.last_relpos_time > 0 else -1.0
            counts = dict(self.ubx_count)

        heading_filtered = circular_mean_deg(list(self.heading_hist)) if self.heading_hist else None
        heading_err = None

        if rel and heading_filtered is not None:
            heading_err = circular_abs_error_deg(rel["heading_deg"], heading_filtered)

        payload = {
            "hpposllh_age_sec": round(hp_age, 2),
            "relposned_age_sec": round(rel_age, 2),
            "pvt_age_sec": round(pvt_age, 2),
            "ubx_count": counts,
            "checksum_fail_count": self.checksum_fail_count,
            "position": hp or {},
            "pvt": pvt or {},
            "relposned": rel or {},
            "heading_filtered_deg": round(heading_filtered, 5)
            if heading_filtered is not None
            else None,
            "heading_filter_error_deg": round(heading_err, 5)
            if heading_err is not None
            else None,
        }

        self.status_pub.publish(String(data=json.dumps(payload, separators=(",", ":"))))

        if hp_age > 3.0 or rel_age > 3.0:
            self.get_logger().warn(
                f"[ROVER] stale data: HPPOSLLH={hp_age:.1f}s RELPOSNED={rel_age:.1f}s",
                throttle_duration_sec=5.0,
            )

    def destroy_node(self):
        self.stop_event.set()
        try:
            if hasattr(self, "ser") and self.ser:
                self.ser.close()
        except Exception:
            pass

        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UbxRoverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ntrip_lite_node.py
────────────────────────────────────────────────────────────────────────────
Lite 보드(Moving Base/Base)에 NTRIP RTCM을 직접 주입하는 ROS 2 노드.

검증된 단독 Python 테스트 구조를 ROS 2 노드로 옮긴 버전입니다.

구조:
  Lite 보드 UART1/USB serial
    ├─ read  : Lite GGA / NAV-PVT 상태 수신
    └─ write : NTRIP caster에서 받은 RTCM frame 직접 주입

발행:
  /gnss/ntrip_status     std_msgs/String       JSON 상태
  /gnss/lite/fix_pvt     sensor_msgs/NavSatFix Lite 보드 PVT 상태 확인용

중요:
  - 이 노드는 Lite 보드 포트만 사용해야 합니다.
  - 큰 보드 Rover 포트는 ubx_rover_node.py가 따로 사용합니다.
  - 큰 보드에는 외부 VRS RTCM을 직접 주입하지 않습니다.
"""

from __future__ import annotations

import base64
import json
import socket
import struct
import threading
import time
from collections import Counter
from datetime import datetime, timezone
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String
import serial


# =============================================================================
# NMEA / UBX / RTCM 유틸
# =============================================================================


def nmea_checksum(body: str) -> str:
    csum = 0
    for ch in body:
        csum ^= ord(ch)
    return f"{csum:02X}"


def validate_nmea_checksum(line: str) -> bool:
    text = line.strip()
    if not text.startswith("$") or "*" not in text:
        return False
    body, given = text[1:].split("*", 1)
    return nmea_checksum(body) == given[:2].upper()


def parse_gga(line: str):
    text = line.strip()
    if not text.startswith("$") or "GGA" not in text[:7]:
        return None

    if "*" in text and not validate_nmea_checksum(text):
        return None

    body = text[1:text.find("*")] if "*" in text else text[1:]
    fields = body.split(",")
    if len(fields) < 10:
        return None

    try:
        quality = int(fields[6]) if fields[6] else 0
    except ValueError:
        quality = 0

    return {
        "raw": text,
        "utc": fields[1],
        "lat_raw": fields[2],
        "ns": fields[3],
        "lon_raw": fields[4],
        "ew": fields[5],
        "quality": quality,
        "num_sv": int(fields[7]) if fields[7].isdigit() else 0,
        "hdop": float(fields[8]) if fields[8] else -1.0,
        "alt": float(fields[9]) if fields[9] else 0.0,
        "valid": bool(fields[2] and fields[3] and fields[4] and fields[5] and quality > 0),
    }


def gga_quality_name(q: int) -> str:
    return {
        0: "INVALID",
        1: "GNSS_FIX",
        2: "DGPS",
        4: "RTK_FIXED",
        5: "RTK_FLOAT",
        6: "DR",
    }.get(q, f"Q{q}")


def deg_to_nmea_lat(lat_deg: float):
    hemi = "N" if lat_deg >= 0 else "S"
    lat_abs = abs(lat_deg)
    deg = int(lat_abs)
    minutes = (lat_abs - deg) * 60.0
    return f"{deg:02d}{minutes:09.6f}", hemi


def deg_to_nmea_lon(lon_deg: float):
    hemi = "E" if lon_deg >= 0 else "W"
    lon_abs = abs(lon_deg)
    deg = int(lon_abs)
    minutes = (lon_abs - deg) * 60.0
    return f"{deg:03d}{minutes:09.6f}", hemi


def make_manual_gga(lat_deg: float, lon_deg: float, alt_m: float) -> bytes:
    now = datetime.now(timezone.utc)
    hhmmss = now.strftime("%H%M%S")
    lat, ns = deg_to_nmea_lat(lat_deg)
    lon, ew = deg_to_nmea_lon(lon_deg)
    body = (
        f"GPGGA,{hhmmss}.00,{lat},{ns},{lon},{ew},"
        f"1,12,1.0,{alt_m:.1f},M,0.0,M,,"
    )
    return f"${body}*{nmea_checksum(body)}\r\n".encode("ascii")


def ubx_checksum(data: bytes) -> Tuple[int, int]:
    ck_a = 0
    ck_b = 0
    for b in data:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return ck_a, ck_b


def extract_rtcm_frames(buf: bytearray):
    frames = []
    while True:
        idx = buf.find(b"\xD3")
        if idx < 0:
            if len(buf) > 4096:
                del buf[:-3]
            break
        if idx > 0:
            del buf[:idx]
        if len(buf) < 3:
            break

        length = ((buf[1] & 0x03) << 8) | buf[2]
        total_len = 3 + length + 3
        if length > 1023:
            del buf[0]
            continue
        if len(buf) < total_len:
            break

        frames.append(bytes(buf[:total_len]))
        del buf[:total_len]
    return frames


def rtcm_message_type(frame: bytes) -> Optional[int]:
    if len(frame) < 6 or frame[0] != 0xD3:
        return None
    length = ((frame[1] & 0x03) << 8) | frame[2]
    payload = frame[3:3 + length]
    if len(payload) < 2:
        return None
    return ((payload[0] << 4) | (payload[1] >> 4)) & 0x0FFF


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


# =============================================================================
# ROS 2 Node
# =============================================================================


class NtripLiteNode(Node):
    def __init__(self):
        super().__init__("ntrip_lite_node")

        # ── serial ────────────────────────────────────────────────────
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)

        # ── NTRIP ─────────────────────────────────────────────────────
        self.declare_parameter("host", "RTS2.ngii.go.kr")
        self.declare_parameter("port", 2101)
        self.declare_parameter("mountpoint", "VRS-RTCM32")
        self.declare_parameter("authenticate", False)
        self.declare_parameter("username", "")
        self.declare_parameter("password", "")

        # ── timing ────────────────────────────────────────────────────
        self.declare_parameter("gga_send_interval_sec", 5.0)
        self.declare_parameter("rtcm_timeout_sec", 35.0)
        self.declare_parameter("reconnect_delay_sec", 5.0)
        self.declare_parameter("status_publish_interval_sec", 2.0)

        # ── fallback ──────────────────────────────────────────────────
        self.declare_parameter("use_manual_gga_fallback", False)
        self.declare_parameter("fallback_lat_deg", 37.3411111111)
        self.declare_parameter("fallback_lon_deg", 126.7317277778)
        self.declare_parameter("fallback_alt_m", 4.0)

        # ── topics ────────────────────────────────────────────────────
        self.declare_parameter("status_topic", "/gnss/ntrip_status")
        self.declare_parameter("lite_fix_topic", "/gnss/lite/fix_pvt")

        self.serial_port = str(self.get_parameter("serial_port").value)
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.host = str(self.get_parameter("host").value)
        self.port = int(self.get_parameter("port").value)
        self.mountpoint = str(self.get_parameter("mountpoint").value)
        self.authenticate = bool(self.get_parameter("authenticate").value)
        self.username = str(self.get_parameter("username").value)
        self.password = str(self.get_parameter("password").value)
        self.gga_send_interval = float(self.get_parameter("gga_send_interval_sec").value)
        self.rtcm_timeout = float(self.get_parameter("rtcm_timeout_sec").value)
        self.reconnect_delay = float(self.get_parameter("reconnect_delay_sec").value)
        self.status_interval = float(self.get_parameter("status_publish_interval_sec").value)
        self.use_manual_gga_fallback = bool(self.get_parameter("use_manual_gga_fallback").value)
        self.fallback_lat = float(self.get_parameter("fallback_lat_deg").value)
        self.fallback_lon = float(self.get_parameter("fallback_lon_deg").value)
        self.fallback_alt = float(self.get_parameter("fallback_alt_m").value)

        self.status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self.lite_fix_pub = self.create_publisher(
            NavSatFix, str(self.get_parameter("lite_fix_topic").value), 10
        )

        self.stop_event = threading.Event()
        self.serial_lock = threading.Lock()
        self.state_lock = threading.Lock()
        self.gga_lock = threading.Lock()

        self.latest_valid_gga: Optional[bytes] = None
        self.latest_gga_text: Optional[str] = None
        self.latest_gga_quality = 0
        self.latest_gga_time = 0.0
        self.latest_pvt = None
        self.latest_pvt_time = 0.0
        self.last_ntrip_rx_time = 0.0
        self.last_rtcm_write_time = 0.0

        self.rtcm_count = Counter()
        self.rtcm_bytes = 0
        self.reconnect_count = 0
        self.ntrip_connected = False
        self.last_error = ""

        try:
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.05,
                write_timeout=1.0,
            )
        except Exception as e:
            self.get_logger().error(f"[LITE] serial open failed: {self.serial_port} - {e}")
            raise

        self.get_logger().info(
            "ntrip_lite_node 시작\n"
            f"  Lite serial : {self.serial_port} @ {self.baudrate}\n"
            f"  NTRIP       : {self.host}:{self.port}/{self.mountpoint}\n"
            f"  timeout     : {self.rtcm_timeout:.1f}s, reconnect {self.reconnect_delay:.1f}s"
        )

        self.reader_thread = threading.Thread(target=self._serial_reader_worker, daemon=True)
        self.ntrip_thread = threading.Thread(target=self._ntrip_worker, daemon=True)
        self.reader_thread.start()
        self.ntrip_thread.start()

        self.create_timer(self.status_interval, self._publish_status)

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
                self._parse_serial_buffer(buf)
            except Exception as e:
                self.last_error = f"serial_read: {e}"
                self.get_logger().warn(f"[LITE] serial read error: {e}", throttle_duration_sec=2.0)
                time.sleep(0.2)

    def _parse_serial_buffer(self, buf: bytearray):
        while buf:
            # UBX
            if len(buf) >= 2 and buf[0] == 0xB5 and buf[1] == 0x62:
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
                    self.get_logger().warn(
                        f"[LITE] UBX checksum fail cls=0x{cls_id:02X} id=0x{msg_id:02X}",
                        throttle_duration_sec=2.0,
                    )
                    continue
                payload = packet[6:6 + length]
                if cls_id == 0x01 and msg_id == 0x07:  # NAV-PVT
                    self._handle_nav_pvt(payload)
                continue

            # NMEA
            if buf[0] == ord("$"):
                end_idx = buf.find(b"\n")
                if end_idx < 0:
                    return
                line = bytes(buf[:end_idx + 1])
                del buf[:end_idx + 1]
                self._handle_nmea(line)
                continue

            # 기타 바이트 제거
            del buf[0]

    def _handle_nmea(self, line: bytes):
        text = line.decode("ascii", errors="replace").strip()
        if "GGA" not in text[:7]:
            return
        info = parse_gga(text)
        if info is None:
            self.get_logger().warn(f"[LITE_GGA] checksum/parse fail: {text}", throttle_duration_sec=3.0)
            return

        with self.gga_lock:
            if info["valid"]:
                self.latest_valid_gga = (text + "\r\n").encode("ascii", errors="ignore")
                self.latest_gga_text = text
                self.latest_gga_quality = int(info["quality"])
                self.latest_gga_time = time.monotonic()

        self.get_logger().info(
            f"[LITE_GGA] {gga_quality_name(info['quality'])} "
            f"SV={info['num_sv']} HDOP={info['hdop']:.2f}",
            throttle_duration_sec=2.0,
        )

    def _handle_nav_pvt(self, payload: bytes):
        pvt = parse_nav_pvt(payload)
        if pvt is None:
            return
        with self.state_lock:
            self.latest_pvt = pvt
            self.latest_pvt_time = time.monotonic()

        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "lite_gps_link"
        msg.latitude = pvt["lat"]
        msg.longitude = pvt["lon"]
        msg.altitude = pvt["alt_m"]
        msg.status.service = NavSatStatus.SERVICE_GPS

        if pvt["carr"] == 2:
            msg.status.status = NavSatStatus.STATUS_GBAS_FIX
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_KNOWN
        elif pvt["carr"] == 1:
            msg.status.status = NavSatStatus.STATUS_GBAS_FIX
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        elif pvt["fix_type"] >= 2:
            msg.status.status = NavSatStatus.STATUS_FIX
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        else:
            msg.status.status = NavSatStatus.STATUS_NO_FIX
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        hvar = pvt["h_acc_m"] ** 2
        vvar = pvt["v_acc_m"] ** 2
        msg.position_covariance = [hvar, 0.0, 0.0, 0.0, hvar, 0.0, 0.0, 0.0, vvar]
        self.lite_fix_pub.publish(msg)

    # ------------------------------------------------------------------
    # NTRIP
    # ------------------------------------------------------------------
    def _get_latest_gga_for_ntrip(self) -> Tuple[Optional[bytes], str]:
        with self.gga_lock:
            if self.latest_valid_gga is not None:
                return self.latest_valid_gga, "SENSOR"
        if self.use_manual_gga_fallback:
            return make_manual_gga(self.fallback_lat, self.fallback_lon, self.fallback_alt), "MANUAL_FALLBACK"
        return None, "NONE"

    def _build_ntrip_request(self) -> bytes:
        mount = self.mountpoint if self.mountpoint.startswith("/") else "/" + self.mountpoint
        lines = [
            f"GET {mount} HTTP/1.0",
            f"Host: {self.host}:{self.port}",
            "User-Agent: MMR-NTRIP-Lite/1.0",
            "Ntrip-Version: Ntrip/2.0",
            "Connection: close",
        ]
        if self.authenticate and (self.username or self.password):
            token = f"{self.username}:{self.password}".encode("utf-8")
            auth = base64.b64encode(token).decode("ascii")
            lines.append(f"Authorization: Basic {auth}")
        lines.extend(["", ""])
        return "\r\n".join(lines).encode("ascii")

    def _wait_valid_gga(self) -> Tuple[bytes, str]:
        while rclpy.ok() and not self.stop_event.is_set():
            gga, src = self._get_latest_gga_for_ntrip()
            if gga is not None:
                return gga, src
            self.get_logger().info("[NTRIP] 유효한 Lite GGA 대기 중", throttle_duration_sec=5.0)
            time.sleep(0.5)
        raise RuntimeError("stopped before valid GGA")

    def _connect_ntrip(self) -> socket.socket:
        gga, src = self._wait_valid_gga()
        self.get_logger().info(f"[NTRIP] connecting {self.host}:{self.port}/{self.mountpoint}, GGA={src}")

        sock = socket.create_connection((self.host, self.port), timeout=10.0)
        sock.settimeout(2.0)
        sock.sendall(self._build_ntrip_request())
        sock.sendall(gga)

        status_line = b""
        while not status_line.endswith(b"\n"):
            b = sock.recv(1)
            if not b:
                raise RuntimeError("NTRIP server closed before status")
            status_line += b
            if len(status_line) > 300:
                break

        status_text = status_line.decode("ascii", errors="replace").strip()
        self.get_logger().info(f"[NTRIP] response: {status_text}")
        if "200" not in status_text and "ICY" not in status_text:
            extra = b""
            try:
                extra = sock.recv(300)
            except Exception:
                pass
            raise RuntimeError(f"NTRIP response failed: {status_text} {extra!r}")

        sock.settimeout(0.2)
        return sock

    def _ntrip_worker(self):
        while rclpy.ok() and not self.stop_event.is_set():
            sock = None
            buf = bytearray()
            last_gga_sent = 0.0
            last_rtcm_frame = time.monotonic()

            try:
                sock = self._connect_ntrip()
                self.ntrip_connected = True
                self.last_error = ""
                last_rtcm_frame = time.monotonic()

                while rclpy.ok() and not self.stop_event.is_set():
                    now = time.monotonic()

                    if now - last_gga_sent >= self.gga_send_interval:
                        gga, src = self._get_latest_gga_for_ntrip()
                        if gga is not None:
                            sock.sendall(gga)
                            last_gga_sent = now
                            self.get_logger().info(f"[NTRIP] GGA sent: {src}", throttle_duration_sec=5.0)

                    if now - last_rtcm_frame > self.rtcm_timeout:
                        raise RuntimeError(f"RTCM timeout {self.rtcm_timeout:.1f}s")

                    try:
                        data = sock.recv(4096)
                    except socket.timeout:
                        continue

                    if not data:
                        raise RuntimeError("NTRIP stream closed")

                    buf.extend(data)
                    frames = extract_rtcm_frames(buf)
                    if frames:
                        last_rtcm_frame = time.monotonic()
                        self.last_ntrip_rx_time = last_rtcm_frame

                    for frame in frames:
                        msg_type = rtcm_message_type(frame)
                        with self.serial_lock:
                            self.ser.write(frame)
                        with self.state_lock:
                            if msg_type is not None:
                                self.rtcm_count[msg_type] += 1
                            self.rtcm_bytes += len(frame)
                            self.last_rtcm_write_time = time.monotonic()

            except Exception as e:
                self.ntrip_connected = False
                self.last_error = str(e)
                self.reconnect_count += 1
                self.get_logger().warn(f"[NTRIP] error: {e}. reconnect after {self.reconnect_delay:.1f}s")
                time.sleep(self.reconnect_delay)
            finally:
                self.ntrip_connected = False
                if sock:
                    try:
                        sock.close()
                    except Exception:
                        pass

    # ------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------
    def _publish_status(self):
        now = time.monotonic()
        with self.gga_lock:
            gga_age = now - self.latest_gga_time if self.latest_gga_time > 0 else None
            gga_quality = self.latest_gga_quality
            gga_text = self.latest_gga_text
        with self.state_lock:
            pvt = self.latest_pvt.copy() if self.latest_pvt else None
            pvt_age = now - self.latest_pvt_time if self.latest_pvt_time > 0 else None
            rtcm_types = dict(self.rtcm_count)
            rtcm_bytes = self.rtcm_bytes
            rtcm_age = now - self.last_rtcm_write_time if self.last_rtcm_write_time > 0 else None

        payload = {
            "ntrip_connected": bool(self.ntrip_connected),
            "last_error": self.last_error,
            "reconnect_count": int(self.reconnect_count),
            "gga_quality": gga_quality_name(gga_quality),
            "gga_age_sec": round(gga_age, 2) if gga_age is not None else -1.0,
            "latest_gga": gga_text or "",
            "rtcm_bytes": int(rtcm_bytes),
            "rtcm_age_sec": round(rtcm_age, 2) if rtcm_age is not None else -1.0,
            "rtcm_types": rtcm_types,
            "pvt": pvt or {},
            "pvt_age_sec": round(pvt_age, 2) if pvt_age is not None else -1.0,
        }
        self.status_pub.publish(String(data=json.dumps(payload, separators=(",", ":"))))

        if rtcm_age is not None and rtcm_age > 10.0:
            self.get_logger().warn(f"[NTRIP] RTCM write age={rtcm_age:.1f}s", throttle_duration_sec=5.0)

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
    node = NtripLiteNode()
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

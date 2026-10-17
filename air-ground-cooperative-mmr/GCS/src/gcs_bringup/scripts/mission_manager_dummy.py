#!/usr/bin/env python3
import json
import time
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MissionManagerDummy(Node):
    def __init__(self):
        super().__init__("mission_manager_dummy")
        self.mission_dir = os.path.expanduser("~/h264_gcs_ws/missions")
        self.pub_status = self.create_publisher(String, "/mission/status", 10)
        self.pub_path = self.create_publisher(String, "/mission/path", 10)

        self.sub_upload = self.create_subscription(
            String,
            "/mission/upload_request",
            self.on_upload_request,
            10
        )

        self.get_logger().info("Mission manager dummy started")

    def publish_status(self, mission_id, state, current_wp=None, total_wp=None, message=""):
        msg = String()
        msg.data = json.dumps({
            "mission_id": mission_id,
            "state": state,
            "current_wp": current_wp,
            "total_wp": total_wp,
            "message": message,
            "stamp": time.time()
        }, ensure_ascii=False)

        self.pub_status.publish(msg)

    def make_dummy_path(self, mission_id):
        """
        Mission Planner .waypoints 파일을 읽어서 UI 표시용 points로 변환.
        QGC WPL 110 형식 기준.

        반환 예시:
        [
            {"seq": 0, "lat": 37.34, "lon": 126.73, "alt": 9.0, "cmd": "TAKEOFF"},
            {"seq": 1, "lat": 37.34, "lon": 126.73, "alt": 9.0, "cmd": "WAYPOINT"},
            ...
        ]
        """
        mission_id = str(mission_id).upper()

        mission_file = os.path.join(
            self.mission_dir,
            f"mission_{mission_id}.waypoints"
        )

        if not os.path.exists(mission_file):
            raise FileNotFoundError(f"mission file not found: {mission_file}")

        cmd_map = {
            16: "WAYPOINT",
            17: "LOITER_UNLIM",
            18: "LOITER_TURNS",
            19: "LOITER_TIME",
            20: "RTL",
            21: "LAND",
            22: "TAKEOFF",
        }

        points = []

        home_lat = None
        home_lon = None
        home_alt = None

        last_valid_lat = None
        last_valid_lon = None
        last_valid_alt = None

        with open(mission_file, "r", encoding="utf-8", errors="ignore") as f:
            lines = f.readlines()

        for line in lines:
            line = line.strip()

            if not line:
                continue

            if line.startswith("QGC"):
                continue

            cols = line.split()

            # QGC WPL 110:
            # seq current frame command param1 param2 param3 param4 lat lon alt autocontinue
            if len(cols) < 12:
                continue

            try:
                seq = int(cols[0])
                current_flag = int(cols[1])
                cmd_num = int(float(cols[3]))
                lat = float(cols[8])
                lon = float(cols[9])
                alt = float(cols[10])
            except Exception:
                continue

            # Mission Planner .waypoints의 0번 행은 HOME/기준 위치.
            # 지도에는 표시하지 않지만, TAKEOFF 좌표 보정용으로 저장한다.
            if seq == 0 and current_flag == 1:
                if abs(lat) > 1e-9 or abs(lon) > 1e-9:
                    home_lat = lat
                    home_lon = lon
                    home_alt = alt
                    self.get_logger().info(
                        f"store Mission Planner HOME row: lat={home_lat}, lon={home_lon}, alt={home_alt}"
                    )
                continue

            # Mission Planner .waypoints 파일의 0번 행은 HOME/기준 위치이므로 UI 표시에서 제외
            if seq == 0 and current_flag == 1:
                self.get_logger().info("skip Mission Planner HOME row: seq 0")
                continue

            cmd = cmd_map.get(cmd_num, f"CMD_{cmd_num}")

            # lat/lon이 0,0인 명령 처리
            # - TAKEOFF는 HOME/H 위치에 표시
            # - LAND/RTL 등은 직전 유효 위치에 표시
            if abs(lat) < 1e-9 and abs(lon) < 1e-9:
                cmd_tmp = cmd_map.get(cmd_num, f"CMD_{cmd_num}").upper()

                if "TAKEOFF" in cmd_tmp and home_lat is not None and home_lon is not None:
                    lat = home_lat
                    lon = home_lon
                    if abs(alt) < 1e-9 and home_alt is not None:
                        alt = home_alt

                elif last_valid_lat is not None and last_valid_lon is not None:
                    lat = last_valid_lat
                    lon = last_valid_lon
                    if abs(alt) < 1e-9 and last_valid_alt is not None:
                        alt = last_valid_alt

            # 그래도 좌표가 0,0이면 UI 지도 표시에서 제외
            if abs(lat) < 1e-9 and abs(lon) < 1e-9:
                self.get_logger().warn(f"skip seq {seq}: invalid lat/lon 0,0")
                continue

            point = {
                "seq": seq,
                "lat": lat,
                "lon": lon,
                "alt": alt,
                "cmd": cmd,
            }

            points.append(point)

            # 유효 좌표 갱신
            last_valid_lat = lat
            last_valid_lon = lon
            last_valid_alt = alt

        if not points:
            raise RuntimeError(f"no valid mission points parsed: {mission_file}")

        self.get_logger().info(
            f"loaded Mission {mission_id}: {mission_file}, points={len(points)}"
        )

        return points

    def publish_path(self, mission_id, current_wp=None):
        points = self.make_dummy_path(mission_id)

        msg = String()
        msg.data = json.dumps({
            "mission_id": mission_id,
            "points": points,
            "current_wp": current_wp,
            "total_wp": len(points),
            "stamp": time.time()
        }, ensure_ascii=False)

        self.pub_path.publish(msg)

    def simulate_mission_progress(self, mission_id):
        points = self.make_dummy_path(mission_id)
        total_wp = len(points)

        # 실제 구현에서는 조종기 AUTO 전환 후 MISSION_CURRENT 수신으로 RUNNING 판단
        time.sleep(1.0)

        for p in points:
            seq = int(p.get("seq", 0))
            cmd = str(p.get("cmd", "-")).upper()

            if "WAYPOINT" in cmd:
                cmd_text = f"WAYPOINT {seq}"
            else:
                cmd_text = cmd

            self.publish_status(
                mission_id,
                "RUNNING",
                current_wp=seq,
                total_wp=total_wp,
                message=f"Mission {mission_id} SEQ{seq} {cmd_text} 수행 중(더미)"
            )

            self.publish_path(mission_id, current_wp=seq)
            time.sleep(1.0)

        last_seq = int(points[-1].get("seq", total_wp - 1))
        last_cmd = str(points[-1].get("cmd", "-")).upper()

        self.publish_status(
            mission_id,
            "COMPLETED",
            current_wp=last_seq,
            total_wp=total_wp,
            message=f"Mission {mission_id} SEQ{last_seq} {last_cmd} 완료(더미)"
        )

    def on_upload_request(self, msg):
        try:
            data = json.loads(msg.data)
            mission_id = str(data.get("mission_id", "UNKNOWN")).upper()

            self.get_logger().info(f"upload request received: Mission {mission_id}")

            points = self.make_dummy_path(mission_id)
            total_wp = len(points)

            self.publish_status(
                mission_id,
                "UPLOADING",
                current_wp=None,
                total_wp=total_wp,
                message=f"Mission {mission_id} 파일 읽는 중"
            )

            time.sleep(0.3)

            self.publish_status(
                mission_id,
                "READY",
                current_wp=None,
                total_wp=total_wp,
                message=f"Mission {mission_id} 파일 로드 완료"
            )

            self.publish_path(mission_id, current_wp=None)

            # 지금은 파일 표시 테스트 단계.
            # 자동 진행 테스트까지 보고 싶으면 아래 줄 유지.
            # 실제 FC 연동 단계에서는 이 줄 제거 예정.
            self.simulate_mission_progress(mission_id)

        except Exception as e:
            self.get_logger().error(f"mission load failed: {e}")

            try:
                mission_id = str(json.loads(msg.data).get("mission_id", "UNKNOWN")).upper()
            except Exception:
                mission_id = "UNKNOWN"

            self.publish_status(
                mission_id,
                "ERROR",
                current_wp=None,
                total_wp=None,
                message=f"Mission {mission_id} 로드 실패: {e}"
            )

def main():
    rclpy.init()
    node = MissionManagerDummy()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
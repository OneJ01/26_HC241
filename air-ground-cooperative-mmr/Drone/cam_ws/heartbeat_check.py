import asyncio
import math
import json
import time
from mavsdk import System

# =========================
# ROS2 publish 추가
# - ROS2 환경이면 /drone/telemetry 로 JSON publish
# - ROS2 환경이 아니면 기존처럼 print-only 동작
# =========================
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    ROS2_AVAILABLE = True
    ROS2_IMPORT_ERROR = None
except Exception as e:
    rclpy = None
    Node = None
    String = None
    ROS2_AVAILABLE = False
    ROS2_IMPORT_ERROR = e

TELEMETRY_TOPIC = "/drone/telemetry"
TELEMETRY_PUB_HZ = 10.0

telemetry_state = {
    "armed": None,
    "flight_mode": None,

    # NOTE:
    # ArduPilot + MAVSDK 조합에서 이 값들은 N/A가 자주 뜸.
    # 지금 단계에서는 UI 핵심값으로 쓰지 말고 보조 확인용으로만 둠.
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

    # UI/HUD에서 바로 쓰기 좋게 수평속도(Ground Speed) 추가
    "ground_speed_m_s": None,

    "gps_fix_type": None,
    "num_satellites": None,

    "battery_v": None,

    # NOTE:
    # ArduPilot에서 remaining_percent가 유효하지 않으면 -1.0 같은 값이 들어올 수 있음.
    # 지금은 신뢰하지 말고, 유효한 경우에만 사용.
    "battery_remaining_pct": None,

    # NOTE:
    # health()는 PX4 쪽에서 더 잘 맞는 편이고,
    # ArduPilot에서는 실제 상태와 다르게 False가 나올 수 있음.
    # 지금은 UI 핵심 판단값으로 쓰지 말고 참고용으로만 둠.
    "health": None,

    "status_text": None,
    "stream_errors": {}
}


def fmt(value, digits=3):
    if value is None:
        return "N/A"
    if isinstance(value, float):
        if math.isnan(value):
            return "N/A"
        return f"{value:.{digits}f}"
    return str(value)


def safe_battery_percent(value):
    """
    배터리 퍼센트 유효성 검사.
    - None / NaN / 음수면 무효 처리
    - 0.0 ~ 1.0 범위만 유효하다고 가정
    """
    if value is None:
        return None
    if isinstance(value, float) and math.isnan(value):
        return None
    if value < 0.0:
        return None
    return value


def compute_ground_speed():
    """
    HUD용 수평속도 계산.
    GS = sqrt(vN^2 + vE^2)
    """
    vn = telemetry_state["vel_n_m_s"]
    ve = telemetry_state["vel_e_m_s"]

    if vn is None or ve is None:
        telemetry_state["ground_speed_m_s"] = None
        return

    try:
        telemetry_state["ground_speed_m_s"] = math.sqrt(vn * vn + ve * ve)
    except Exception:
        telemetry_state["ground_speed_m_s"] = None


def build_ui_payload():
    """
    UI/ROS2 publish로 바로 넘길 핵심 payload.
    """
    batt_pct = safe_battery_percent(telemetry_state["battery_remaining_pct"])

    return {
        "armed": telemetry_state["armed"],
        "flight_mode": telemetry_state["flight_mode"],

        "roll_deg": telemetry_state["roll_deg"],
        "pitch_deg": telemetry_state["pitch_deg"],
        "yaw_deg": telemetry_state["yaw_deg"],

        "lat_deg": telemetry_state["lat_deg"],
        "lon_deg": telemetry_state["lon_deg"],

        "abs_alt_m": telemetry_state["abs_alt_m"],
        "rel_alt_m": telemetry_state["rel_alt_m"],

        "vel_n_m_s": telemetry_state["vel_n_m_s"],
        "vel_e_m_s": telemetry_state["vel_e_m_s"],
        "vel_d_m_s": telemetry_state["vel_d_m_s"],
        "ground_speed_m_s": telemetry_state["ground_speed_m_s"],

        "gps_fix_type": telemetry_state["gps_fix_type"],
        "num_satellites": telemetry_state["num_satellites"],

        "battery_v": telemetry_state["battery_v"],
        "battery_remaining_pct": batt_pct,

        "status_text": telemetry_state["status_text"],

        # NOTE:
        # 아래 값들은 지금은 UI 핵심값으로 쓰지 않는 것을 권장
        "in_air": telemetry_state["in_air"],
        "landed_state": telemetry_state["landed_state"],
        "health": telemetry_state["health"],
    }


async def safe_runner(name, coro):
    try:
        await coro
    except Exception as e:
        telemetry_state["stream_errors"][name] = str(e)
        print(f"[WARN] {name} stopped: {e}")


async def watch_armed(drone):
    async for armed in drone.telemetry.armed():
        telemetry_state["armed"] = armed


async def watch_flight_mode(drone):
    async for flight_mode in drone.telemetry.flight_mode():
        telemetry_state["flight_mode"] = str(flight_mode)


async def watch_in_air(drone):
    async for in_air in drone.telemetry.in_air():
        telemetry_state["in_air"] = in_air


async def watch_landed_state(drone):
    async for landed_state in drone.telemetry.landed_state():
        telemetry_state["landed_state"] = str(landed_state)


async def watch_euler_angle(drone):
    async for euler in drone.telemetry.attitude_euler():
        telemetry_state["roll_deg"] = euler.roll_deg
        telemetry_state["pitch_deg"] = euler.pitch_deg
        telemetry_state["yaw_deg"] = euler.yaw_deg


async def watch_position(drone):
    async for position in drone.telemetry.position():
        telemetry_state["lat_deg"] = position.latitude_deg
        telemetry_state["lon_deg"] = position.longitude_deg
        telemetry_state["abs_alt_m"] = position.absolute_altitude_m
        telemetry_state["rel_alt_m"] = position.relative_altitude_m


async def watch_velocity_ned(drone):
    async for vel in drone.telemetry.velocity_ned():
        telemetry_state["vel_n_m_s"] = vel.north_m_s
        telemetry_state["vel_e_m_s"] = vel.east_m_s
        telemetry_state["vel_d_m_s"] = vel.down_m_s

        # HUD용 수평속도 즉시 갱신
        compute_ground_speed()


async def watch_gps_info(drone):
    async for gps in drone.telemetry.gps_info():
        telemetry_state["gps_fix_type"] = str(gps.fix_type)
        telemetry_state["num_satellites"] = gps.num_satellites


async def watch_battery(drone):
    async for battery in drone.telemetry.battery():
        telemetry_state["battery_v"] = battery.voltage_v
        telemetry_state["battery_remaining_pct"] = safe_battery_percent(
            battery.remaining_percent
        )


async def watch_health(drone):
    async for health in drone.telemetry.health():
        telemetry_state["health"] = {
            "is_gyrometer_calibration_ok": getattr(health, "is_gyrometer_calibration_ok", None),
            "is_accelerometer_calibration_ok": getattr(health, "is_accelerometer_calibration_ok", None),
            "is_magnetometer_calibration_ok": getattr(health, "is_magnetometer_calibration_ok", None),
            "is_local_position_ok": getattr(health, "is_local_position_ok", None),
            "is_global_position_ok": getattr(health, "is_global_position_ok", None),
            "is_home_position_ok": getattr(health, "is_home_position_ok", None),
            "is_armable": getattr(health, "is_armable", None),
        }


async def watch_status_text(drone):
    async for status_text in drone.telemetry.status_text():
        telemetry_state["status_text"] = f"[{status_text.type}] {status_text.text}"


async def request_telemetry_rates(drone):
    """
    ArduPilot 쪽에 각 telemetry stream rate 요청
    """
    try:
        await drone.telemetry.set_rate_attitude_euler(10.0)
        print("[INFO] set_rate_attitude_euler(10) OK")
    except Exception as e:
        print(f"[WARN] set_rate_attitude_euler failed: {e}")

    try:
        await drone.telemetry.set_rate_position(10.0)
        print("[INFO] set_rate_position(10) OK")
    except Exception as e:
        print(f"[WARN] set_rate_position failed: {e}")

    try:
        await drone.telemetry.set_rate_velocity_ned(10.0)
        print("[INFO] set_rate_velocity_ned(10) OK")
    except Exception as e:
        print(f"[WARN] set_rate_velocity_ned failed: {e}")

    try:
        await drone.telemetry.set_rate_gps_info(2.0)
        print("[INFO] set_rate_gps_info(2) OK")
    except Exception as e:
        print(f"[WARN] set_rate_gps_info failed: {e}")

    try:
        await drone.telemetry.set_rate_battery(2.0)
        print("[INFO] set_rate_battery(2) OK")
    except Exception as e:
        print(f"[WARN] set_rate_battery failed: {e}")

    try:
        await drone.telemetry.set_rate_status_text(2.0)
        print("[INFO] set_rate_status_text(2) OK")
    except Exception as e:
        print(f"[WARN] set_rate_status_text failed: {e}")


# =========================
# ROS2 publish 추가
# =========================
class TelemetryPublisher(Node):
    def __init__(self):
        super().__init__("heartbeat_telemetry_pub")
        self.pub = self.create_publisher(String, TELEMETRY_TOPIC, 10)
        self.get_logger().info(f"Publishing telemetry to {TELEMETRY_TOPIC}")


async def publish_loop(ros_node):
    """
    build_ui_payload() 결과를 JSON 문자열로 ROS2 publish.
    UI는 이 토픽을 subscribe해서 HUD를 갱신하면 됨.
    """
    period = 1.0 / TELEMETRY_PUB_HZ

    while True:
        try:
            payload = build_ui_payload()
            payload["telem_time_ns"] = time.time_ns()

            msg = String()
            msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
            ros_node.pub.publish(msg)
        except Exception as e:
            print(f"[WARN] ROS2 publish failed: {e}")

        await asyncio.sleep(period)


async def print_loop():
    while True:
        print("\n" + "=" * 60)
        print("Telemetry Snapshot (ArduPilot / Mission Planner)")
        print("-" * 60)

        print(f"Armed               : {fmt(telemetry_state['armed'])}")
        print(f"FlightMode          : {fmt(telemetry_state['flight_mode'])}")

        # NOTE:
        # 아래 두 값은 현재 ArduPilot+MAVSDK에서 신뢰도가 낮아
        # 앞으로도 UI 핵심 표시값으로는 당분간 제외 권장.
        print(f"InAir               : {fmt(telemetry_state['in_air'])}")
        print(f"LandedState         : {fmt(telemetry_state['landed_state'])}")

        print(
            f"Roll/Pitch/Yaw(deg) : "
            f"{fmt(telemetry_state['roll_deg'])}, "
            f"{fmt(telemetry_state['pitch_deg'])}, "
            f"{fmt(telemetry_state['yaw_deg'])}"
        )

        print(
            f"Lat/Lon             : "
            f"{fmt(telemetry_state['lat_deg'], 7)}, "
            f"{fmt(telemetry_state['lon_deg'], 7)}"
        )

        print(
            f"Abs/Rel Alt(m)      : "
            f"{fmt(telemetry_state['abs_alt_m'])}, "
            f"{fmt(telemetry_state['rel_alt_m'])}"
        )

        print(
            f"Vel N/E/D (m/s)     : "
            f"{fmt(telemetry_state['vel_n_m_s'])}, "
            f"{fmt(telemetry_state['vel_e_m_s'])}, "
            f"{fmt(telemetry_state['vel_d_m_s'])}"
        )

        print(f"Ground Speed(m/s)   : {fmt(telemetry_state['ground_speed_m_s'])}")

        print(
            f"GPS Fix / Sats      : "
            f"{fmt(telemetry_state['gps_fix_type'])}, "
            f"{fmt(telemetry_state['num_satellites'])}"
        )

        batt_pct = telemetry_state["battery_remaining_pct"]
        batt_pct_str = "N/A" if batt_pct is None else f"{batt_pct * 100:.1f}%"

        print(f"Battery(V / %)      : {fmt(telemetry_state['battery_v'])}, {batt_pct_str}")
        print(f"StatusText          : {fmt(telemetry_state['status_text'])}")

        health = telemetry_state["health"]
        if health is None:
            print("Health              : N/A")
        else:
            print(
                "Health              : "
                f"gyro={fmt(health['is_gyrometer_calibration_ok'])}, "
                f"accel={fmt(health['is_accelerometer_calibration_ok'])}, "
                f"mag={fmt(health['is_magnetometer_calibration_ok'])}, "
                f"local_pos={fmt(health['is_local_position_ok'])}, "
                f"global_pos={fmt(health['is_global_position_ok'])}, "
                f"home={fmt(health['is_home_position_ok'])}, "
                f"armable={fmt(health['is_armable'])}"
            )

        if telemetry_state["stream_errors"]:
            print("StreamErrors        :", telemetry_state["stream_errors"])

        # DEBUG:
        # UI로 실제 보내는 payload 확인용
        ui_payload = build_ui_payload()
        print("UI Payload          :", json.dumps(ui_payload, ensure_ascii=False))

        await asyncio.sleep(1.0)


async def main():
    ros_node = None

    # =========================
    # ROS2 init
    # - ROS2 환경이 아니면 기존처럼 print-only
    # =========================
    if ROS2_AVAILABLE:
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
            ros_node = TelemetryPublisher()
            print(f"[INFO] ROS2 publisher ready: {TELEMETRY_TOPIC}")
        except Exception as e:
            ros_node = None
            print(f"[WARN] ROS2 init failed -> print-only mode: {e}")
    else:
        print(f"[WARN] ROS2 not available -> print-only mode: {ROS2_IMPORT_ERROR}")

    drone = System()

    print("Connecting to FC on /dev/ttyACM0 ...")
    await drone.connect(system_address="serial:///dev/ttyACM0:115200")

    print("Waiting for connection (heartbeat)...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected: heartbeat received")
            break

    # 연결 직후 autopilot이 stream 준비할 시간을 조금 줌
    await asyncio.sleep(2.0)

    print("Requesting telemetry rates...")
    await request_telemetry_rates(drone)

    # rate 요청 후 반영 시간 조금 대기
    await asyncio.sleep(2.0)

    tasks = [
        asyncio.create_task(safe_runner("armed", watch_armed(drone))),
        asyncio.create_task(safe_runner("flight_mode", watch_flight_mode(drone))),
        asyncio.create_task(safe_runner("in_air", watch_in_air(drone))),
        asyncio.create_task(safe_runner("landed_state", watch_landed_state(drone))),
        asyncio.create_task(safe_runner("attitude_euler", watch_euler_angle(drone))),
        asyncio.create_task(safe_runner("position", watch_position(drone))),
        asyncio.create_task(safe_runner("velocity_ned", watch_velocity_ned(drone))),
        asyncio.create_task(safe_runner("gps_info", watch_gps_info(drone))),
        asyncio.create_task(safe_runner("battery", watch_battery(drone))),
        asyncio.create_task(safe_runner("health", watch_health(drone))),
        asyncio.create_task(safe_runner("status_text", watch_status_text(drone))),
        asyncio.create_task(print_loop()),
    ]

    # ROS2가 준비됐으면 publish loop 추가
    if ros_node is not None:
        tasks.append(asyncio.create_task(publish_loop(ros_node)))

    try:
        await asyncio.gather(*tasks)
    finally:
        if ros_node is not None:
            try:
                ros_node.destroy_node()
            except Exception:
                pass

        if ROS2_AVAILABLE:
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    asyncio.run(main())
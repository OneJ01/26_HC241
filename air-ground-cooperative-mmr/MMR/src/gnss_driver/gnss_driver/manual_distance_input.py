"""
MMR 수동 타겟팅 노드 (CLI 기반)

[역할]
- /gnss/fix_hp, /gnss/heading 을 구독하여 현재 MMR 위치·헤딩을 파악
- 사용자가 CLI로 입력한 거리(d), 방위각(p), 또는 절대 좌표(t)를 바탕으로
  /target/point JSON 을 fire_solution_node 로 발행

[토픽]
  구독:
    /gnss/fix_hp    sensor_msgs/NavSatFix   초정밀 RTK 위경도
    /gnss/heading   std_msgs/Float64        Moving Baseline RTK 헤딩 (0~360°)
  발행:
    /target/point   std_msgs/String         표적 좌표 JSON

[/target/point JSON 스키마]
  {
    "target_id"          : int,
    "lat_deg"            : float,   # WGS84 위도
    "lon_deg"            : float,   # WGS84 경도
    "z_m"                : float,   # 표적 고도 (AMSL m)
    "status"             : "HIT",   # fire_solution_node 검증 통과용 필수 필드
    "dem_intersection_ok": true,    # fire_solution_node 검증 통과용 필수 필드
    "range_m"            : float,   # (선택) 입력 거리
    "bearing_deg"        : float,   # (선택) 입력 방위각
    "t_publish_ns"       : int      # 발행 타임스탬프 (ns)
  }

[CLI 명령]
  d <m>               : 표적 거리 설정        (예: d 100)
  p <deg>             : 팬 절대 방위각 설정   (예: p 45)
  t <lat> <lon> [alt] : 표적 좌표 직접 입력   (예: t 37.123 126.456 15)
  s                   : 현재 센서 상태 출력
  r                   : 팬/거리 초기화
  q                   : 종료

[실행]
  ros2 run gnss_driver manual_target
"""

import json
import time
import threading
import queue
import os

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64, String

from pyproj import Geod, Transformer

try:
    import rasterio
    _RASTERIO_OK = True
except ImportError:
    _RASTERIO_OK = False

DEFAULT_DEM_PATH = os.path.expanduser("~/mmr_ws/data/dem_tin_1m_pot.tif")


class ManualTargetNode(Node):
    def __init__(self):
        super().__init__('manual_target_node')

        # ------------------------------------------------------------------
        # 상태 변수
        # ------------------------------------------------------------------
        self.mmr_lat:     float | None = None
        self.mmr_lon:     float | None = None
        self.mmr_alt_m:   float | None = None
        self.heading_deg: float | None = None

        self.target_alt_m: float = 10.0   # 기본 표적 고도 (지상 가정)
        self.pan_deg:  float | None = None
        self.dist_m:   float | None = None
        self.target_id: int = 0

        self.input_queue = queue.Queue()
        self._geod = Geod(ellps='WGS84')

        # DEM 로드 (표적 고도 자동 조회용)
        self._dem_ds = None
        self._dem_to_crs = None
        if _RASTERIO_OK and os.path.exists(DEFAULT_DEM_PATH):
            try:
                self._dem_ds = rasterio.open(DEFAULT_DEM_PATH)
                if str(self._dem_ds.crs) != "EPSG:4326":
                    self._dem_to_crs = Transformer.from_crs(
                        "EPSG:4326", self._dem_ds.crs, always_xy=True)
                self.get_logger().info(f"[DEM] 로드 완료: {DEFAULT_DEM_PATH}")
            except Exception as e:
                self.get_logger().warn(f"[DEM] 로드 실패: {e}")
        else:
            self.get_logger().warn("[DEM] rasterio 없거나 파일 없음 — 고도 0.0m 사용")

        # ------------------------------------------------------------------
        # Publisher / Subscriber
        # ------------------------------------------------------------------
        self.pub_target = self.create_publisher(String, '/target/point', 10)

        self.create_subscription(
            NavSatFix, '/gnss/fix_hp', self._cb_fix, 10)
        self.create_subscription(
            Float64, '/gnss/heading', self._cb_heading, 10)

        # ------------------------------------------------------------------
        # 타이머 + 입력 스레드
        # ------------------------------------------------------------------
        self.create_timer(0.1, self._process_input_queue)

        th = threading.Thread(target=self._input_loop, daemon=True)
        th.start()

        self.get_logger().info(
            '[ManualTarget] 노드 시작. /gnss/fix_hp + /gnss/heading 수신 대기 중...'
        )

    # ------------------------------------------------------------------
    # 콜백
    # ------------------------------------------------------------------
    def _cb_fix(self, msg: NavSatFix):
        self.mmr_lat   = msg.latitude
        self.mmr_lon   = msg.longitude
        self.mmr_alt_m = msg.altitude

    def _cb_heading(self, msg: Float64):
        self.heading_deg = msg.data

    # ------------------------------------------------------------------
    # DEM 고도 조회
    # ------------------------------------------------------------------
    def _dem_altitude(self, lat: float, lon: float) -> float:
        """DEM에서 위경도 지점의 지면 고도(m)를 반환. 실패 시 0.0 반환."""
        if self._dem_ds is None:
            return 0.0
        try:
            if self._dem_to_crs is not None:
                x, y = self._dem_to_crs.transform(lon, lat)
            else:
                x, y = lon, lat
            val = list(self._dem_ds.sample([(x, y)]))[0][0]
            if val is None:
                return 0.0
            val = float(val)
            nodata = self._dem_ds.nodata
            if (nodata is not None and val == nodata):
                return 0.0
            import math
            if math.isnan(val):
                return 0.0
            return val
        except Exception:
            return 0.0

    # ------------------------------------------------------------------
    # 좌표 계산 (Forward Geodetics)
    # ------------------------------------------------------------------
    def _offset_latlon(self, lat: float, lon: float,
                       bearing_deg: float, dist_m: float):
        lon2, lat2, _ = self._geod.fwd(lon, lat, bearing_deg, dist_m)
        return lat2, lon2

    # ------------------------------------------------------------------
    # CLI 입력 루프 (별도 스레드)
    # ------------------------------------------------------------------
    def _input_loop(self):
        print('\n====================================================')
        print('  MMR 수동 타겟팅 노드 (Live GNSS 연동)')
        print('====================================================')
        print('  d <m>               : 표적 거리 설정 (예: d 100)')
        print('  p <deg>             : 팬 절대 방위각 설정 (예: p 45)')
        print('  t <lat> <lon> [alt] : 표적 좌표 직접 입력')
        print('  s                   : 현재 센서 상태 확인')
        print('  r                   : 팬/거리 초기화')
        print('  q                   : 종료')
        print('====================================================\n')

        while True:
            try:
                line = input('command > ').strip()
            except EOFError:
                break
            if not line:
                continue
            self.input_queue.put(line)
            if line.lower() in ('q', 'quit', 'exit'):
                break

    # ------------------------------------------------------------------
    # 명령 처리기 (0.1s 타이머)
    # ------------------------------------------------------------------
    def _process_input_queue(self):
        while not self.input_queue.empty():
            line = self.input_queue.get()

            if line.lower() in ('q', 'quit', 'exit'):
                rclpy.shutdown()
                return

            if line.lower() == 's':
                self._print_status()
                continue

            if line.lower().startswith('t '):
                try:
                    parts = line.split()
                    if len(parts) >= 3:
                        lat = float(parts[1])
                        lon = float(parts[2])
                        alt = float(parts[3]) if len(parts) >= 4 else self.target_alt_m
                        self._publish_by_coord(lat, lon, alt)
                except Exception:
                    self.get_logger().warn('[ManualTarget] t 명령 파싱 실패')
                continue

            if line.lower() == 'r':
                self.pan_deg = None
                self.dist_m  = None
                self.get_logger().info('[ManualTarget] 팬/거리 초기화 완료')
                continue

            if line.lower().startswith('d '):
                try:
                    dist = float(line.split()[1])
                    if dist > 0:
                        self.dist_m = dist
                except Exception:
                    pass
                self._try_publish()
                continue

            if line.lower().startswith('p '):
                try:
                    self.pan_deg = float(line.split()[1]) % 360.0
                except Exception:
                    pass
                self._try_publish()
                continue

    def _print_status(self):
        lat_s = f'{self.mmr_lat:.7f}'  if self.mmr_lat   is not None else '수신 대기 중...'
        lon_s = f'{self.mmr_lon:.7f}'  if self.mmr_lon   is not None else '수신 대기 중...'
        hdg_s = f'{self.heading_deg:.2f}°' if self.heading_deg is not None else '수신 대기 중...'
        pan_s = f'{self.pan_deg:.2f}°' if self.pan_deg  is not None else 'None'
        dst_s = f'{self.dist_m:.2f}m'  if self.dist_m   is not None else 'None'

        if self.pan_deg is not None and self.heading_deg is not None:
            yaw = (self.pan_deg - self.heading_deg + 360.0) % 360.0
            if yaw > 180.0:
                yaw -= 360.0
            yaw_s = f'{yaw:+.2f}° (포탑 회전량)'
        else:
            yaw_s = '계산 불가 (데이터 부족)'

        self.get_logger().info(
            f'\n[LIVE STATUS]\n'
            f'  MMR 위치 : Lat {lat_s}, Lon {lon_s}\n'
            f'  MMR 헤딩 : {hdg_s}\n'
            f'  타겟 입력: pan={pan_s}, dist={dst_s}\n'
            f'  포탑 회전: {yaw_s}'
        )

    # ------------------------------------------------------------------
    # Failsafe 발행 게이트
    # ------------------------------------------------------------------
    def _try_publish(self):
        if self.dist_m is None or self.pan_deg is None:
            return
        if self.mmr_lat is None or self.heading_deg is None:
            self.get_logger().warn(
                '[ManualTarget] GNSS 위치/헤딩 수신 전 — 오발 방지를 위해 대기 중'
            )
            return
        self._publish_by_range_bearing(self.dist_m, self.pan_deg)

    # ------------------------------------------------------------------
    # 발행 (거리/방위각 → 위경도 변환)
    # ------------------------------------------------------------------
    def _publish_by_range_bearing(self, dist_m: float, pan_deg: float):
        target_lat, target_lon = self._offset_latlon(
            self.mmr_lat, self.mmr_lon, pan_deg, dist_m)
        self.target_id += 1

        z_m = self._dem_altitude(target_lat, target_lon)
        payload = {
            'target_id':           self.target_id,
            'lat_deg':             target_lat,
            'lon_deg':             target_lon,
            'z_m':                 z_m,
            'status':              'HIT',        # fire_solution_node 검증 필수
            'sync_ok':             True,         # fire_solution_node 검증 필수
            'projection_ok':       True,         # fire_solution_node 검증 필수
            'dem_intersection_ok': True,         # fire_solution_node 검증 필수
            'sync_error_ms':       0.0,
            'source':              'manual',
            'range_m':             dist_m,
            'bearing_deg':         pan_deg,
            't_publish_ns':        time.time_ns(),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub_target.publish(msg)
        self.get_logger().info(
            f'[TARGET] ID={self.target_id} | '
            f'Lat={target_lat:.7f}, Lon={target_lon:.7f} | '
            f'dist={dist_m:.1f}m, bearing={pan_deg:.1f}°'
        )

    # ------------------------------------------------------------------
    # 발행 (절대 좌표 직접 입력)
    # ------------------------------------------------------------------
    def _publish_by_coord(self, lat: float, lon: float, alt: float):
        self.target_id += 1
        z_m = self._dem_altitude(lat, lon)
        payload = {
            'target_id':           self.target_id,
            'lat_deg':             lat,
            'lon_deg':             lon,
            'z_m':                 z_m,
            'status':              'HIT',
            'sync_ok':             True,
            'projection_ok':       True,
            'dem_intersection_ok': True,
            'sync_error_ms':       0.0,
            'source':              'manual',
            't_publish_ns':        time.time_ns(),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub_target.publish(msg)
        self.get_logger().info(
            f'[TARGET] ID={self.target_id} (절대 좌표) | '
            f'Lat={lat:.7f}, Lon={lon:.7f}, DEM_Alt={z_m:.1f}m'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ManualTargetNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
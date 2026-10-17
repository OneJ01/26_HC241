import os
import socket
import time
from pymavlink import mavutil

os.environ["MAVLINK20"] = "1"

TCP_HOST = "127.0.0.1"
TCP_PORT = 2102

FC_PORT = "/dev/serial/by-id/usb-Koadrone_KoaFC_0-if00"
FC_BAUD = 2000000

# msg233 payload는 최대 180 bytes
CHUNK = 180

SEND_HZ = 20.0
MIN_DT = 1.0 / SEND_HZ
last_send = 0.0

HB_HZ = 1.0
HB_DT = 1.0 / HB_HZ
last_hb = 0.0

# ✅ msg233 fragment id/sequence 관리
seq = 0  # 0~31 순환 사용(5bit)

def send_gcs_heartbeat(m):
    m.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0, 0,
        mavutil.mavlink.MAV_STATE_ACTIVE
    )

def main():
    global last_send, last_hb, seq

    print("[1] MAVLink 연결...")
    m = mavutil.mavlink_connection(
        FC_PORT, baud=FC_BAUD,
        source_system=200, source_component=191,
        robust_parsing=True, autoreconnect=True
    )
    m.wait_heartbeat(timeout=15)
    print("[OK] Heartbeat from FC")

    send_gcs_heartbeat(m)
    last_hb = time.time()

    print("[2] TCP 연결(str2str 출력)...")
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((TCP_HOST, TCP_PORT))
    s.settimeout(2.0)
    print("[OK] TCP connected")

    while True:
        # 1Hz heartbeat 유지
        now = time.time()
        if now - last_hb >= HB_DT:
            send_gcs_heartbeat(m)
            last_hb = now

        try:
            data = s.recv(4096)
            if not data:
                time.sleep(0.01)
                continue
        except socket.timeout:
            continue

        # msg233로 전송
        for off in range(0, len(data), CHUNK):
            chunk = data[off:off+CHUNK]
            payload = chunk + bytes(180 - len(chunk))  # 180 고정 배열

            # 속도 제한
            now = time.time()
            dt = now - last_send
            if dt < MIN_DT:
                time.sleep(MIN_DT - dt)
            last_send = time.time()

            # ✅ flags: 상위 3bit는 reserved, 하위 5bit를 sequence로 쓰는 관례가 많음
            # PX4가 어떤 조합을 기대할지 애매해서, "seq만 넣는" 가장 단순한 형태부터 시도
            flags = 0  # ✅ fragmentation 안 쓰기!
            m.mav.gps_rtcm_data_send(flags, len(chunk), payload)

if __name__ == "__main__":
    main()
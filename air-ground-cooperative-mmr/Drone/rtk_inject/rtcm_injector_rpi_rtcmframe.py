import os
import socket
import time
from pymavlink import mavutil

os.environ["MAVLINK20"] = "1"

TCP_HOST = "127.0.0.1"
TCP_PORT = 2102

FC_PORT = "/dev/serial/by-id/usb-Koadrone_KoaFC_0-if00"
FC_BAUD = 2000000

# MAVLink GPS_RTCM_DATA 한 패킷당 데이터 최대
MAV233_MAX = 180

# 송신 제한(너무 빡세면 RTK 수렴이 느려지고, 너무 높으면 loss↑)
SEND_HZ = 30.0
MIN_DT = 1.0 / SEND_HZ
last_send = 0.0

HB_HZ = 1.0
HB_DT = 1.0 / HB_HZ
last_hb = 0.0

seq = 0  # 0~31

def send_gcs_heartbeat(m):
    m.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0, 0,
        mavutil.mavlink.MAV_STATE_ACTIVE
    )

def send_rtcm233(m, frame: bytes):
    """
    frame: RTCM3 프레임 전체(0xD3 포함, CRC 3바이트 포함)
    - 180바이트 이하면 flags=0으로 1패킷
    - 초과하면 flags로 fragment/seq 지정해서 분할
    """
    global last_send, seq

    n = len(frame)
    if n <= MAV233_MAX:
        payload = frame + bytes(MAV233_MAX - n)
        # fragmentation 안 씀
        flags = 0
        _rate_limit()
        m.mav.gps_rtcm_data_send(flags, n, payload)
        return

    # fragmentation 사용 (frag_id 2비트 → 최대 4조각)
    # 180*4=720B까지 처리 가능(대부분 RTCM은 여기 안 넘음)
    max_frags = 4
    needed = (n + MAV233_MAX - 1) // MAV233_MAX
    if needed > max_frags:
        # 너무 큰 프레임이면 그냥 드랍(현실적으로 거의 없음)
        return

    for frag_id in range(needed):
        start = frag_id * MAV233_MAX
        end = min(start + MAV233_MAX, n)
        chunk = frame[start:end]
        payload = chunk + bytes(MAV233_MAX - len(chunk))

        # flags 비트 구성:
        # bit0 = 1 (fragmented)
        # bit1~2 = frag_id (0~3)
        # bit3~7 = seq (0~31)
        flags = 1 | ((frag_id & 0x03) << 1) | ((seq & 0x1F) << 3)

        _rate_limit()
        m.mav.gps_rtcm_data_send(flags, len(chunk), payload)

    seq = (seq + 1) % 32

def _rate_limit():
    global last_send
    now = time.time()
    dt = now - last_send
    if dt < MIN_DT:
        time.sleep(MIN_DT - dt)
    last_send = time.time()

def extract_rtcm3_frames(buf: bytearray):
    frames = []
    while True:
        # 0xD3 찾기
        idx = buf.find(0xD3)
        if idx < 0:
            # 프리앰블이 TCP 경계에서 잘릴 수 있으니 끝 2바이트만 남김
            if len(buf) > 2:
                del buf[:-2]
            break

        # 앞 쓰레기 제거
        if idx > 0:
            del buf[:idx]

        # 헤더 3바이트 필요
        if len(buf) < 3:
            break

        b1 = buf[1]
        b2 = buf[2]
        payload_len = ((b1 & 0x03) << 8) | b2
        frame_len = 3 + payload_len + 3  # header + payload + CRC

        if len(buf) < frame_len:
            break

        frame = bytes(buf[:frame_len])
        frames.append(frame)
        del buf[:frame_len]

    return frames
    return frames

def main():
    global last_hb

    print("[1] MAVLink 연결...")
    m = mavutil.mavlink_connection(
        FC_PORT, baud=FC_BAUD,
        source_system=200, source_component=191,
        robust_parsing=True, autoreconnect=True
    )
    m.wait_heartbeat(timeout=15)
    print("[OK] Heartbeat from FC")

    # 우리가 GCS라는 heartbeat 송신 시작
    send_gcs_heartbeat(m)
    last_hb = time.time()

    print("[2] TCP 연결(str2str 출력)...")
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((TCP_HOST, TCP_PORT))
    s.settimeout(2.0)
    print("[OK] TCP connected")

    buf = bytearray()
    last_print = time.time()
    recv_bytes = 0
    sent_frames = 0

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

        buf.extend(data)
        recv_bytes += len(data)

        frames = extract_rtcm3_frames(buf)
        print(f"[DBG] buf_len={len(buf)} d3_count={buf.count(0xD3)} frames={len(frames)}")
        for fr in frames:
            send_rtcm233(m, fr)
            sent_frames += 1

        if time.time() - last_print > 2.0:
            print(f"[RTCM] recv={recv_bytes}B, frames_sent={sent_frames}")
            last_print = time.time()

if __name__ == "__main__":
    main()
import serial

PORT = "/dev/serial0"
BAUD = 115200

ser = serial.Serial(
    port=PORT,
    baudrate=BAUD,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=1
)

print(f"opened: {PORT} @ {BAUD}")

while True:
    first = ser.read(1)
    if not first:
        continue

    if first[0] != 0x59:
        continue

    second = ser.read(1)
    if not second or second[0] != 0x59:
        continue

    rest = ser.read(7)   # 총 9바이트 프레임
    if len(rest) != 7:
        continue

    frame = bytes([0x59, 0x59]) + rest

    dist = frame[2] | (frame[3] << 8)
    strength = frame[4] | (frame[5] << 8)
    temp_raw = frame[6] | (frame[7] << 8)

    print(f"dist={dist} cm, strength={strength}, temp_raw={temp_raw}")
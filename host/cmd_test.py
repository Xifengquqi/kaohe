"""下发指令测试：设置目标角 / 在线改 PID，并读取 ACK

用法：
    py host/cmd_test.py                 # 默认 COM7，把目标角设为 0/0
    py host/cmd_test.py COM5 10 -5      # 指定串口，目标 pitch=10° yaw=-5°

协议见 docs/02-通信协议.md：
    AA 55 | LEN | CMD | DATA | CRC16(大端，覆盖 LEN..DATA)
"""
import sys
import struct
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM7"
TARGET_PITCH = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
TARGET_YAW = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build(cmd: int, payload: bytes) -> bytes:
    body = bytes([len(payload), cmd]) + payload
    c = crc16(body)
    return b"\xAA\x55" + body + bytes([(c >> 8) & 0xFF, c & 0xFF])


def send_target(ser, pitch_deg: float, yaw_deg: float):
    payload = struct.pack("<hh", int(pitch_deg * 100), int(yaw_deg * 100))
    ser.write(build(0x10, payload))


def send_pid(ser, kp: float, ki: float, kd: float):
    payload = struct.pack("<hhh", int(kp * 100), int(ki * 100), int(kd * 100))
    ser.write(build(0x11, payload))


def wait_ack(ser, timeout=1.0):
    """返回 (被回应的命令, ok) 或 None"""
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < timeout:
        buf += ser.read(64)
        while True:
            i = buf.find(b"\xAA\x55")
            if i < 0:
                if len(buf) > 1:
                    buf = buf[-1:]
                break
            if len(buf) < i + 4:
                break
            ln = buf[i + 2]
            if len(buf) < i + 6 + ln:
                break
            fr = buf[i:i + 6 + ln]
            if crc16(fr[2:4 + ln]) == ((fr[4 + ln] << 8) | fr[5 + ln]) and fr[3] == 0x04:
                return (fr[4], fr[5])
            buf = buf[i + 6 + ln:]
    return None


def main():
    with serial.Serial(PORT, 115200, timeout=0.1) as ser:
        ser.reset_input_buffer()

        print(f"[1] 设置目标角 pitch={TARGET_PITCH}° yaw={TARGET_YAW}°")
        send_target(ser, TARGET_PITCH, TARGET_YAW)
        ack = wait_ack(ser)
        print("    收到 ACK:", ack if ack else "没有（检查板子是否在跑、串口是否接对）")

        print("[2] 设置 PID: Kp=4.0 Ki=0.3 Kd=0.0")
        send_pid(ser, 4.0, 0.3, 0.0)
        ack = wait_ack(ser)
        print("    收到 ACK:", ack if ack else "没有")


if __name__ == "__main__":
    main()

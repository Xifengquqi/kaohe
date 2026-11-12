# -*- coding: utf-8 -*-
"""
串口测试: 接收下位机发来的姿态帧, 打印 pitch/roll/yaw

用法:
    1. 插上 CH340, 设备管理器里看是 COM 几, 改下面的 PORT
    2. py host/serial_test.py
"""

import struct
import sys
import time

import serial

PORT = "COM7"          # ← 改成你的端口号
BAUD = 115200

FRAME_HEAD = b"\xAA\x55"
CMD_ATTITUDE = 0x01


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def main() -> int:
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.5)
    except Exception as e:
        print(f"打不开 {PORT}: {e}")
        print("检查: 1) CH340 插了吗 2) 端口号对吗 3) 有没有被串口助手占用")
        return 1

    buf = bytearray()
    ok = 0
    bad = 0
    t0 = time.time()

    print(f"已连接 {PORT} @ {BAUD}, 等待数据... (Ctrl+C 退出)")

    try:
        while True:
            buf += ser.read(64)

            while True:
                i = buf.find(FRAME_HEAD)
                if i < 0:
                    if len(buf) > 1:
                        buf = buf[-1:]
                    break
                if len(buf) < i + 4:
                    break

                ln = buf[i + 2]
                if len(buf) < i + 6 + ln:
                    break

                frame = buf[i:i + 6 + ln]
                calc = crc16(frame[2:4 + ln])
                rx = (frame[4 + ln] << 8) | frame[5 + ln]

                if calc == rx and frame[3] == CMD_ATTITUDE and ln == 8:
                    p, r, y, seq = struct.unpack("<hhhh", bytes(frame[4:12]))
                    ok += 1
                    print(f"pitch={p/100:7.2f}  roll={r/100:7.2f}  "
                          f"yaw={y/100:7.2f}  seq={seq}")
                else:
                    bad += 1

                buf = buf[i + 6 + ln:]

    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        dt = time.time() - t0
        print(f"\n共收到 {ok} 帧, 校验失败 {bad} 次, 用时 {dt:.1f}s "
              f"({ok/dt if dt > 0 else 0:.1f} 帧/秒)")
    return 0


if __name__ == "__main__":
    sys.exit(main())

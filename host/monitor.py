# -*- coding: utf-8 -*-
"""
云台上位机 v0.1
- 读取串口姿态帧 (AA 55 LEN CMD DATA CRC)
- 实时画 pitch / roll / yaw 三条曲线
- 显示帧率、校验错误数

用法: py host/monitor.py
"""

import struct
import sys
import time
from collections import deque

import serial
from PyQt5 import QtCore, QtWidgets
import pyqtgraph as pg

PORT = "COM7"          # ← 改成你的端口
BAUD = 115200

MAX_POINTS = 1500      # 每条曲线保留多少点 (10Hz 下约 150 秒)
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


class SerialReader(QtCore.QThread):
    """串口读取线程: 收字节、拆帧、发信号给界面"""

    frame_ready = QtCore.pyqtSignal(float, float, float)
    error = QtCore.pyqtSignal(str)

    def __init__(self, port: str, baud: int, parent=None):
        super().__init__(parent)
        self.port = port
        self.baud = baud
        self._running = True

    def stop(self):
        self._running = False

    def run(self):
        try:
            ser = serial.Serial(self.port, self.baud, timeout=0.2)
        except Exception as e:
            self.error.emit(f"打开 {self.port} 失败: {e}")
            return

        buf = bytearray()
        while self._running:
            try:
                chunk = ser.read(64)
            except Exception as e:
                self.error.emit(f"串口读取出错: {e}")
                break
            if not chunk:
                continue
            buf += chunk

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

                fr = buf[i:i + 6 + ln]
                calc = crc16(fr[2:4 + ln])
                rx = (fr[4 + ln] << 8) | fr[5 + ln]

                if calc == rx and fr[3] == CMD_ATTITUDE and ln == 8:
                    p, r, y, _seq = struct.unpack("<hhhh", bytes(fr[4:12]))
                    self.frame_ready.emit(p / 100.0, r / 100.0, y / 100.0)

                buf = buf[i + 6 + ln:]

        ser.close()


class Monitor(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("云台姿态监视器")
        self.resize(1000, 600)

        pg.setConfigOptions(antialias=True)

        self.plot = pg.PlotWidget()
        self.plot.setBackground("w")
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.setLabel("left", "角度", units="deg")
        self.plot.setLabel("bottom", "时间", units="s")
        self.plot.addLegend()
        self.setCentralWidget(self.plot)

        self.curve_p = self.plot.plot(pen=pg.mkPen("#d62728", width=2), name="pitch")
        self.curve_r = self.plot.plot(pen=pg.mkPen("#2ca02c", width=2), name="roll")
        self.curve_y = self.plot.plot(pen=pg.mkPen("#1f77b4", width=2), name="yaw")

        self.t0 = time.time()
        self.ts = deque(maxlen=MAX_POINTS)
        self.ps = deque(maxlen=MAX_POINTS)
        self.rs = deque(maxlen=MAX_POINTS)
        self.ys = deque(maxlen=MAX_POINTS)

        self.lbl = QtWidgets.QLabel("等待数据...")
        self.lbl.setStyleSheet("font-size:18px; padding:6px;")
        self.statusBar().addPermanentWidget(self.lbl, 1)

        self.frames = 0
        self.fps = 0.0
        self._last_fps_t = time.time()
        self._last_fps_n = 0

        self.reader = SerialReader(PORT, BAUD, self)
        self.reader.frame_ready.connect(self.on_frame)
        self.reader.error.connect(self.on_error)
        self.reader.start()

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(30)  # 界面 33Hz 刷新, 比数据 10Hz 快就够

    @QtCore.pyqtSlot(float, float, float)
    def on_frame(self, p, r, y):
        self.frames += 1
        t = time.time() - self.t0
        self.ts.append(t)
        self.ps.append(p)
        self.rs.append(r)
        self.ys.append(y)

        now = time.time()
        dt = now - self._last_fps_t
        if dt >= 1.0:
            self.fps = (self.frames - self._last_fps_n) / dt
            self._last_fps_t = now
            self._last_fps_n = self.frames

    @QtCore.pyqtSlot(str)
    def on_error(self, msg):
        QtWidgets.QMessageBox.critical(self, "串口错误", msg)

    def refresh(self):
        if not self.ts:
            return
        self.curve_p.setData(list(self.ts), list(self.ps))
        self.curve_r.setData(list(self.ts), list(self.rs))
        self.curve_y.setData(list(self.ts), list(self.ys))

        self.lbl.setText(
            f"pitch={self.ps[-1]:7.2f}  roll={self.rs[-1]:7.2f}  "
            f"yaw={self.ys[-1]:7.2f}   共 {self.frames} 帧  {self.fps:.1f} 帧/秒"
        )

    def closeEvent(self, event):
        self.reader.stop()
        self.reader.wait(1000)
        event.accept()


def main():
    app = QtWidgets.QApplication(sys.argv)
    w = Monitor()
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

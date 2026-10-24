# -*- coding: utf-8 -*-
"""
ESP32-S3 三通道实时波形上位机
    CH1 : ADS1292 呼吸通道 (增益2, ProtoCentral 板上定义)
    CH2 : ADS1292 心电主通道 (增益6, LA/RA 电极, 波形与BPM看这张)
    PPG : MAX30102 红外光电容积波

串口帧格式（16 字节，小端）:
    AA 55 | CH1 int32 | CH2 int32 | PPG int32 | LOFF u8 | SUM u8
    SUM = sum(b[2..14]) & 0xFF

用法:
    python ecg_host.py                # 自动查找 ESP32 串口
    python ecg_host.py COM6           # 指定串口
    python ecg_host.py COM6 115200    # 指定串口与波特率

界面说明:
    顶部工具条  串口选择 / 刷新 / 连接 / 显示模式 / 自动缩放 / 演示模式
    中间三张图  从上到下 CH1(呼吸)、CH2(心电)、PPG，横轴为最近 6 秒，右端是当前时刻
    底部诊断栏  帧率、已收帧数、坏帧数、各通道原始整数、LOFF 电极状态

没有信号时先看底部诊断栏:
    - “未连接 / 没收到数据”      -> 板子没插好，或串口选错、被别的程序占用
    - 帧率 0 但有字节             -> 固件不是最新版（还在发文本日志）
    - 帧率 ≈ 500、原始值在跳、    -> 数据链路正常，波形平说明电极没接好
      但图是平的                    （看底部 LOFF：红点代表该电极脱落）
"""
import sys
import time

import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg
import serial
from serial.tools import list_ports

# ----------------------------------------------------------------------------
# 常量
# ----------------------------------------------------------------------------
FS         = 500          # 采样率(Hz)，需与固件 CONFIG1 的 500SPS 一致
WINDOW_SEC = 6            # 显示窗口(秒)
N          = FS * WINDOW_SEC
FRAME_LEN  = 16
HDR0, HDR1 = 0xAA, 0x55
BAUD_LIST  = ("115200", "460800", "921600", "1500000")

CH_ORDER = ("ch1", "ch2", "ppg")
CH_COLOR = {"ch1": "#3ddc84", "ch2": "#4fc3f7", "ppg": "#ff6b6b"}
CH_TITLE = {"ch1": "CH1  呼吸/辅助通道 (增益2)",
            "ch2": "CH2  心电主通道 (导联I, 增益6)",
            "ppg": "PPG  (MAX30102 红外)"}
LOFF_NAME = ("IN1P", "IN1N", "IN2P", "IN2N")


def find_port():
    """优先选 ESP32 原生 USB-Serial/JTAG(VID 303A)，其次常见 USB 转串口"""
    ports = list(list_ports.comports())
    for p in ports:
        if p.vid == 0x303A:
            return p.device
    for p in ports:
        if p.vid in (0x1A86, 0x10C4, 0x0403):
            return p.device
    return ports[0].device if ports else None


# ----------------------------------------------------------------------------
# 数据链路
# ----------------------------------------------------------------------------
class FrameParser:
    """按帧头 + 校验和解析，可容忍夹在帧之间的文本日志与丢字节"""

    def __init__(self):
        self.buf = bytearray()
        self.good = 0
        self.bad = 0

    def reset(self):
        self.buf.clear()

    def feed(self, data):
        self.buf += data
        out = []
        while True:
            i = self.buf.find(b"\xAA\x55")
            if i < 0:
                # 没找到帧头，只保留最后 1 字节（可能是半截帧头）
                if len(self.buf) > 1:
                    del self.buf[:-1]
                break
            if i:
                del self.buf[:i]
            if len(self.buf) < FRAME_LEN:
                break
            f = self.buf[:FRAME_LEN]
            if (sum(f[2:15]) & 0xFF) == f[15]:
                self.good += 1
                out.append((int.from_bytes(f[2:6], "little", signed=True),
                            int.from_bytes(f[6:10], "little", signed=True),
                            int.from_bytes(f[10:14], "little", signed=True),
                            f[14]))
                del self.buf[:FRAME_LEN]
            else:
                # 校验失败：说明这个"帧头"是数据里凑巧出现的，前进 1 字节重找
                self.bad += 1
                del self.buf[:1]
        return out


class DCBlock:
    """一阶高通，滤掉电极直流失调，让波形落在屏幕中间"""

    def __init__(self, fs, fc=0.5):
        self.a = float(np.exp(-2.0 * np.pi * fc / fs))
        self.dc = 0.0

    def reset(self):
        self.dc = 0.0

    def __call__(self, x):
        self.dc = self.a * self.dc + (1.0 - self.a) * x
        return x - self.dc


class Ring:
    """定长环形缓冲，view() 返回按时间先后排列的副本"""

    def __init__(self, n):
        self.n = n
        self.buf = np.zeros(n, dtype=np.float32)
        self.i = 0

    def clear(self):
        self.buf[:] = 0.0

    def push(self, v):
        self.buf[self.i] = v
        self.i = (self.i + 1) % self.n

    def view(self):
        return np.concatenate((self.buf[self.i:], self.buf[:self.i]))


class DemoSource:
    """内置演示信号：验证上位机渲染是否正常（不连接任何硬件）"""

    def __init__(self, fs):
        self.fs = fs
        self.t = 0.0
        self.acc = 0.0
        self.rr = 0.85          # R-R 间期 → 约 70BPM

    def gen(self, dt):
        """按经过时间 dt 生成若干采样点"""
        out = []
        self.acc += dt
        step = 1.0 / self.fs
        while self.acc >= step:
            self.acc -= step
            self.t += step
            out.append((self._ecg(0.0), self._ecg(0.18), self._ppg()))
        return out

    def _beat(self, ph):
        """单个心拍模板（高斯组合），ph 已归一化到 0~1"""
        return (0.10 * np.exp(-((ph - 0.10) ** 2) / (2 * 0.018 ** 2))
                - 0.12 * np.exp(-((ph - 0.19) ** 2) / (2 * 0.007 ** 2))
                + 1.00 * np.exp(-((ph - 0.20) ** 2) / (2 * 0.009 ** 2))
                - 0.28 * np.exp(-((ph - 0.22) ** 2) / (2 * 0.010 ** 2))
                + 0.32 * np.exp(-((ph - 0.38) ** 2) / (2 * 0.035 ** 2)))

    def _ecg(self, delay):
        ph = ((self.t - delay) % self.rr) / self.rr
        return 260000.0 * self._beat(ph)

    def _ppg(self):
        ph = (self.t % self.rr) / self.rr
        v = 1.00 * np.exp(-((ph - 0.16) ** 2) / (2 * 0.055 ** 2))
        v += 0.42 * np.exp(-((ph - 0.42) ** 2) / (2 * 0.075 ** 2))
        return 90000.0 * v + 120000.0


# ----------------------------------------------------------------------------
# 主窗口
# ----------------------------------------------------------------------------
class WaveWindow(QtWidgets.QMainWindow):
    def __init__(self, port, baud):
        super().__init__()
        self.setWindowTitle("ESP32-S3  心电 / 血氧波形上位机")
        self.resize(1240, 860)

        pg.setConfigOptions(antialias=True, background="#0f1115",
                            foreground="#c8ccd4")

        self.parser = FrameParser()
        self.ring = {k: Ring(N) for k in CH_ORDER}
        self.dcb = {k: DCBlock(FS, 0.5) for k in CH_ORDER}
        self.x = np.arange(N, dtype=np.float32)

        self.ser = None
        self.port = port
        self.baud = baud
        self.auto_conn = True       # 掉线后自动重连
        self.raw_mode = False       # False=去直流显示, True=原始整数
        self.autoscale = True
        self.demo = False
        self.demo_src = DemoSource(FS)
        self._demo_last = 0.0

        self.frames = 0             # 本统计周期内新帧数
        self.total = 0              # 累计帧数
        self.fps = 0.0
        self.t0 = time.time()
        self.last_rx = 0.0
        self.loff = 0
        self.sat = False
        self.noads = False
        self.last_raw = {k: 0 for k in CH_ORDER}
        self.bad_ecg = 0            # 被丢弃的"无应答"帧数（CH1/CH2 无效）
        self.ecg_peak = 1
        self.ecg_base = None
        self.bpm = 0
        self._n = 0                 # 累计**有效**心电采样点数（用于 R-R 间期）
        self._last_beat_n = 0

        self._build_ui()

        # ---- 定时器 ----
        self.t_read = QtCore.QTimer(self)
        self.t_read.timeout.connect(self.on_read)
        self.t_read.start(10)

        self.t_plot = QtCore.QTimer(self)
        self.t_plot.timeout.connect(self.on_plot)
        self.t_plot.start(33)

        self.t_rng = QtCore.QTimer(self)
        self.t_rng.timeout.connect(self.on_range)
        self.t_rng.start(500)

        self.t_demo = QtCore.QTimer(self)
        self.t_demo.timeout.connect(self.on_demo)
        self.t_demo.setInterval(10)

        self.t_reconn = QtCore.QTimer(self)
        self.t_reconn.timeout.connect(self.on_reconnect)
        self.t_reconn.start(2000)

        # 启动即尝试连接
        self.refresh_ports()
        if port:
            self.connect_port(port, baud)

    # ---------------------------------------------------------------- UI
    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        lay = QtWidgets.QVBoxLayout(central)
        lay.setContentsMargins(8, 8, 8, 8)
        lay.setSpacing(6)

        # 工具条
        bar = QtWidgets.QHBoxLayout()
        bar.setSpacing(8)
        bar.addWidget(QtWidgets.QLabel("串口"))
        self.cmb_port = QtWidgets.QComboBox()
        self.cmb_port.setMinimumWidth(150)
        bar.addWidget(self.cmb_port)
        bar.addWidget(QtWidgets.QLabel("波特率"))
        self.cmb_baud = QtWidgets.QComboBox()
        self.cmb_baud.addItems(BAUD_LIST)
        self.cmb_baud.setCurrentText("115200")
        bar.addWidget(self.cmb_baud)

        self.btn_refresh = QtWidgets.QPushButton("刷新")
        self.btn_refresh.clicked.connect(self.refresh_ports)
        bar.addWidget(self.btn_refresh)

        self.btn_conn = QtWidgets.QPushButton("连接")
        self.btn_conn.clicked.connect(self.on_conn_clicked)
        bar.addWidget(self.btn_conn)

        bar.addSpacing(16)
        bar.addWidget(QtWidgets.QLabel("显示"))
        self.cmb_mode = QtWidgets.QComboBox()
        self.cmb_mode.addItems(["去直流 DC", "原始 RAW"])
        self.cmb_mode.currentIndexChanged.connect(self.on_mode_changed)
        bar.addWidget(self.cmb_mode)

        self.chk_auto = QtWidgets.QCheckBox("自动纵向缩放")
        self.chk_auto.setChecked(True)
        self.chk_auto.stateChanged.connect(self.on_autoscale_changed)
        bar.addWidget(self.chk_auto)

        bar.addSpacing(16)
        self.chk_demo = QtWidgets.QCheckBox("演示模式")
        self.chk_demo.setToolTip("不接硬件，生成标准心电/脉搏波形，用于确认上位机本身正常")
        self.chk_demo.stateChanged.connect(self.on_demo_changed)
        bar.addWidget(self.chk_demo)

        bar.addStretch(1)
        self.lbl_conn = QtWidgets.QLabel("未连接")
        self.lbl_conn.setStyleSheet("font-size:13px; color:#ff5252;")
        bar.addWidget(self.lbl_conn)
        lay.addLayout(bar)

        # 三张图
        self.gw = pg.GraphicsLayoutWidget()
        self.plots = {}
        self.curves = {}
        for row, k in enumerate(CH_ORDER):
            p = self.gw.addPlot(row=row, col=0, title=CH_TITLE[k])
            p.showGrid(x=True, y=True, alpha=0.15)
            p.setXRange(0, N, padding=0)
            p.setMouseEnabled(x=False, y=True)
            p.setMenuEnabled(False)
            p.enableAutoRange(axis="y", enable=False)
            p.getAxis("bottom").setTicks(
                [[(0, "-6s"), (N // 2, "-3s"), (N, "0s")]])
            p.setLabel("left", "")
            self.curves[k] = p.plot(pen=pg.mkPen(CH_COLOR[k], width=1.4))
            self.plots[k] = p
        for row in (0, 1):
            self.gw.ci.layout.setRowStretchFactor(row, 1)
        lay.addWidget(self.gw, 1)

        # 诊断栏
        self.loff_label = QtWidgets.QLabel()
        self.loff_label.setTextFormat(QtCore.Qt.RichText)
        self.status = QtWidgets.QLabel("等待数据 ...")
        self.status.setTextFormat(QtCore.Qt.RichText)
        self.status.setStyleSheet("color:#c8ccd4; font-size:13px;")
        lay.addWidget(self.loff_label)
        lay.addWidget(self.status)

    # ------------------------------------------------------------- 串口
    def refresh_ports(self):
        want = self.cmb_port.currentText()
        self.cmb_port.clear()
        found = list(list_ports.comports())
        for p in found:
            desc = f"{p.device}  ({p.description})" if p.description else p.device
            self.cmb_port.addItem(desc, p.device)
        if not found:
            self.cmb_port.addItem("（未发现串口）", None)
        # 保持当前选择
        for i in range(self.cmb_port.count()):
            if self.cmb_port.itemData(i) == want:
                self.cmb_port.setCurrentIndex(i)
                break
        else:
            if self.port:
                for i in range(self.cmb_port.count()):
                    if self.cmb_port.itemData(i) == self.port:
                        self.cmb_port.setCurrentIndex(i)
                        break

    def connect_port(self, port, baud):
        if self.demo:
            return True
        self.close_port()
        try:
            ser = serial.Serial(port, baud, timeout=0)
        except Exception as e:
            self.set_conn_state(False, f"串口打开失败: {e}")
            return False
        # 关键：ESP32-S3 原生 USB 口打开后若 DTR/RTS 为高，
        # 芯片会被按在复位态、一个字都不发。必须拉低。
        try:
            ser.dtr = False
            ser.rts = False
        except Exception:
            pass
        try:
            ser.set_buffer_size(rx_size=1 << 16, tx_size=4096)
        except Exception:
            pass
        self.ser = ser
        self.port = port
        self.baud = baud
        self.parser.reset()
        self.set_conn_state(True, f"已连接 {port} @ {baud}")
        return True

    def close_port(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def set_conn_state(self, ok, text):
        self.lbl_conn.setText(text)
        self.lbl_conn.setStyleSheet(
            "font-size:13px; color:%s;" % ("#3ddc84" if ok else "#ff5252"))
        self.btn_conn.setText("断开" if (self.ser or self.demo) else "连接")

    def on_conn_clicked(self):
        if self.demo:
            return
        if self.ser is not None:
            self.auto_conn = False
            self.close_port()
            self.set_conn_state(False, "已断开")
            return
        self.auto_conn = True
        dev = self.cmb_port.currentData()
        baud = int(self.cmb_baud.currentText())
        if not dev:
            self.refresh_ports()
            dev = self.cmb_port.currentData()
        if dev:
            self.connect_port(dev, baud)

    def on_reconnect(self):
        if self.demo or self.ser is not None or not self.auto_conn:
            return
        dev = self.cmb_port.currentData() or find_port()
        if dev:
            self.connect_port(dev, int(self.cmb_baud.currentText()))

    # --------------------------------------------------------- 模式切换
    def on_mode_changed(self, idx):
        self.raw_mode = (idx == 1)
        for k in CH_ORDER:
            self.dcb[k].reset()
            self.ring[k].clear()
        self.ecg_base = None

    def on_autoscale_changed(self, state):
        self.autoscale = bool(state)
        for k in CH_ORDER:
            self.plots[k].enableAutoRange(axis="y", enable=False)
        if not self.autoscale:
            for k in CH_ORDER:
                self.plots[k].setMouseEnabled(x=False, y=True)

    def on_demo_changed(self, state):
        self.demo = bool(state)
        if self.demo:
            self.auto_conn = False
            self.close_port()
            self._demo_last = time.time()
            self.t_demo.start()
            self.set_conn_state(True, "演示模式（无硬件）")
        else:
            self.t_demo.stop()
            self.auto_conn = True
            self.set_conn_state(False, "未连接")
            self.on_reconnect()

    # ------------------------------------------------------------ 数据
    def _push(self, ch1, ch2, ppg, loff):
        self.frames += 1
        self.total += 1
        self.loff = loff & 0x1F        # bit4=RLD；bit7=ADC 饱和；bit6=心电模块无应答
        self.sat = bool(loff & 0x80)
        self.noads = bool(loff & 0x40)
        self.last_raw = {"ch1": ch1, "ch2": ch2, "ppg": ppg}

        # PPG 来自 MAX30102，独立于心电模块，任何一帧都是有效的
        self.ring["ppg"].push(ppg if self.raw_mode else self.dcb["ppg"](ppg))

        # 无应答帧里的 CH1/CH2 是固件填的 0，不是真实心电数据：
        # 直接丢掉，既不进波形也不参与 R 波检测。否则这些 0 与偶发的
        # 有效值交替，去直流后就会冒出"孤立尖峰 + 指数拖尾"，BPM 也会
        # 被这些跳变顶成乱值。丢掉后波形会诚实地停在原位（平线）。
        if self.noads:
            self.bad_ecg += 1
            return

        self._n += 1
        for k, v in (("ch1", ch1), ("ch2", ch2)):
            self.ring[k].push(v if self.raw_mode else self.dcb[k](v))
        # CH2 的 R 波检测（用于显示 BPM，与固件 OLED 上的一致口径）。
        # 心电主通道是 CH2（增益6）；CH1 是呼吸通道，不参与 BPM。
        if not self.raw_mode:
            v = self.ring["ch2"].buf[(self.ring["ch2"].i - 1) % N]
            if self.ecg_base is None:
                self.ecg_base = v
            self.ecg_base += (v - self.ecg_base) * 0.02
            d = abs(v - self.ecg_base)
            # 幅度包络：涨得快、落得慢。衰减必须足够慢（按采样点算），
            # 否则两个心拍之间包络掉到 0，噪声会把 BPM 顶到很高。
            if d > self.ecg_peak:
                self.ecg_peak += (d - self.ecg_peak) * 0.08
            else:
                self.ecg_peak -= (self.ecg_peak - d) * 0.0025
            if self.ecg_peak < 1000:
                self.ecg_peak = 1000        # 幅度下限，防止静默期乱触发
            # 用采样点计数算 R-R 间期，不受串口到达抖动影响
            if d > self.ecg_peak * 0.55 and \
                    (self._n - self._last_beat_n) > int(0.28 * FS):
                if self._last_beat_n:
                    rr = (self._n - self._last_beat_n) / FS
                    if rr > 0:
                        self.bpm = int(60.0 / rr)
                self._last_beat_n = self._n

    def on_read(self):
        if self.ser is None:
            return
        try:
            data = self.ser.read(4096)
        except Exception as e:
            # 设备被拔掉 / 端口异常
            self.close_port()
            self.set_conn_state(False, f"串口异常: {e}")
            return
        if not data:
            return
        self.last_rx = time.time()
        for ch1, ch2, ppg, loff in self.parser.feed(data):
            self._push(ch1, ch2, ppg, loff)

    def on_demo(self):
        now = time.time()
        dt = now - self._demo_last
        self._demo_last = now
        # 单次最多补 0.2 秒，避免定时器卡顿后一次性灌入过多数据
        dt = min(dt, 0.2)
        for ch1, ch2, ppg in self.demo_src.gen(dt):
            # 加一点噪声，更接近真实
            ch1 += np.random.randn() * 3000
            ch2 += np.random.randn() * 3000
            self._push(int(ch1), int(ch2), int(ppg), 0x00)
        self.last_rx = now

    # ------------------------------------------------------------ 显示
    def on_plot(self):
        for k in CH_ORDER:
            self.curves[k].setData(self.x, self.ring[k].view())
            cur = self.last_raw[k]
            tag = "" if self.raw_mode else "  (显示已去直流)"
            self.plots[k].setTitle(CH_TITLE[k] + f"    当前原始值 = {cur}{tag}",
                                   color="#c8ccd4", size="10pt")

        dt = time.time() - self.t0
        if dt >= 1.0:
            self.fps = self.frames / dt
            self.frames = 0
            self.t0 = time.time()
        self.update_status()

    def on_range(self):
        if not self.autoscale:
            return
        for k in CH_ORDER:
            y = self.ring[k].view()
            lo, hi = float(np.percentile(y, 1)), float(np.percentile(y, 99))
            span = hi - lo
            mid = 0.5 * (lo + hi)
            if span < 20.0:
                span = 20.0
            if span > 2.0e7:
                span = 2.0e7
            self.plots[k].setYRange(mid - span * 0.7, mid + span * 0.7,
                                    padding=0)

    def update_status(self):
        if self.demo:
            head = '<span style="color:#4fc3f7">演示模式</span>'
        elif self.ser is None:
            head = '<span style="color:#ff5252">未连接</span>'
        else:
            head = '<span style="color:#3ddc84">已连接</span> ' + str(self.port)

        no_data = (time.time() - self.last_rx) > 2.0
        if no_data:
            tip = ("　<span style='color:#ffb300'>2 秒内没有收到任何字节</span>"
                   "（检查板子是否插好 / 串口是否被占用 / 是否已烧录最新固件）")
        elif self.parser.good == 0 and self.total == 0:
            tip = ("　<span style='color:#ffb300'>收到字节但解析不出数据帧</span>"
                   "（固件可能还是旧的文本版本）")
        else:
            tip = ""

        sat_txt = ('　<span style="color:#ff5252;font-weight:bold">'
                   'ADC 已饱和（±8388607）→ 电极悬空或超量程</span>'
                   if self.sat else "")
        if self.noads:
            sat_txt += ('　<span style="color:#ff5252;font-weight:bold">'
                        'ADS1292 无应答（ID≠0x73）→ 模块供电 5V / SPI 接线</span>')

        self.status.setText(
            f"{head}　帧率 <b>{self.fps:5.1f}</b> Hz　"
            f"累计帧 {self.total}　坏帧 {self.parser.bad}　"
            f"无效心电帧 {self.bad_ecg}　"
            f"CH1={self.last_raw['ch1']} CH2={self.last_raw['ch2']} "
            f"PPG={self.last_raw['ppg']}　LOFF=0x{self.loff:02X}"
            f"　BPM≈{'—' if self.noads else self.bpm}{sat_txt}{tip}")

        if self.noads:
            self.loff_label.setText(
                '<span style="color:#ff5252;font-weight:bold">'
                'ADS1292 无应答，导联脱落状态无效</span>'
                '（模块必须接 5V 供电，开发板 5V 脚是输入不能当输出）')
            return

        if self.demo or self.ser is not None:
            parts = []
            for i, nm in enumerate(LOFF_NAME):
                if (self.loff >> i) & 1:
                    parts.append(f'<span style="color:#ff5252">●</span> {nm} 脱落')
                else:
                    parts.append(f'<span style="color:#3ddc84">●</span> {nm} 已接')
            if (self.loff & 0x03) == 0x03:
                ch1 = ('<span style="color:#ff5252;font-weight:bold">'
                       'CH1 两个电极都没接上 → 波形只能是悬空噪声</span>')
            elif (self.loff & 0x03) == 0x00:
                ch1 = ('<span style="color:#3ddc84;font-weight:bold">'
                       'CH1 电极已接好</span>')
            else:
                ch1 = ('<span style="color:#ffb300;font-weight:bold">'
                       'CH1 电极接触不良</span>')
            self.loff_label.setText("　".join(parts) + "　　" + ch1)
        else:
            self.loff_label.setText("")

    def closeEvent(self, ev):
        self.close_port()
        ev.accept()


def main():
    argv = [a for a in sys.argv[1:] if a != "--demo"]
    demo = len(argv) != len(sys.argv[1:])
    port = argv[0] if len(argv) > 0 else find_port()
    baud = int(argv[1]) if len(argv) > 1 else 115200
    if not port and not demo:
        print("没有找到串口，请用: python ecg_host.py COM6")
    app = QtWidgets.QApplication([sys.argv[0]])
    w = WaveWindow(port, baud)
    if demo:
        w.chk_demo.setChecked(True)
    w.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
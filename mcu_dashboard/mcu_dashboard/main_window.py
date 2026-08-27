"""PyQt5 main window for telemetry and motor command testing."""

import numpy as np
import pyqtgraph as pg
from PyQt5 import QtCore, QtGui, QtWidgets

from .model import depth_from_pressure


AXIS_COLORS = ('#d1495b', '#16836b', '#2b6cb0')
GROUP_NAMES = ('第 1 组', '第 2 组', '第 3 组', '第 4 组')


class StatusBadge(QtWidgets.QLabel):
    COLORS = {
        'ok': ('#e7f5ef', '#12624d', '#9dd8c3'),
        'warn': ('#fff4d8', '#7a5100', '#e6c56e'),
        'error': ('#fdebec', '#922b35', '#e5a0a7'),
        'idle': ('#edf0f2', '#53616b', '#ccd3d8'),
    }

    def __init__(self, text):
        super().__init__(text)
        self.setAlignment(QtCore.Qt.AlignCenter)
        self.setMinimumHeight(28)
        self.setMinimumWidth(92)
        self.set_state(text, 'idle')

    def set_state(self, text, state):
        background, foreground, border = self.COLORS[state]
        self.setText(text)
        self.setStyleSheet(
            'QLabel {'
            f'background: {background}; color: {foreground}; border: 1px solid {border};'
            'border-radius: 5px; padding: 4px 9px; font-weight: 600;'
            '}'
        )


class Readout(QtWidgets.QFrame):
    def __init__(self, label, unit, accent):
        super().__init__()
        self.setObjectName('readout')
        self.display_unit = unit
        self.setMinimumHeight(92)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(12, 9, 12, 9)
        layout.setSpacing(1)

        title = QtWidgets.QLabel(label)
        title.setObjectName('readoutTitle')
        self.value = QtWidgets.QLabel('--')
        self.value.setStyleSheet(f'color: {accent}; font-size: 25px; font-weight: 650;')
        layout.addWidget(title)
        layout.addWidget(self.value)

    def set_value(self, value, decimals):
        self.value.setText(f'{value:.{decimals}f} {self.display_unit}')

    def clear(self):
        self.value.setText('--')


class MotorChannel(QtWidgets.QFrame):
    value_changed = QtCore.pyqtSignal(float)

    def __init__(self, index, zero_icon):
        super().__init__()
        self.setObjectName('motorChannel')
        self.setFixedHeight(60)
        layout = QtWidgets.QGridLayout(self)
        layout.setContentsMargins(8, 3, 8, 3)
        layout.setHorizontalSpacing(6)
        layout.setVerticalSpacing(0)

        title = QtWidgets.QLabel(f'电机 {index + 1}')
        title.setObjectName('motorTitle')
        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider.setRange(-1000, 1000)
        self.slider.setSingleStep(10)
        self.slider.setPageStep(100)
        self.slider.setValue(0)
        self.slider.setMinimumWidth(120)

        self.spin = QtWidgets.QDoubleSpinBox()
        self.spin.setRange(-1.0, 1.0)
        self.spin.setDecimals(3)
        self.spin.setSingleStep(0.05)
        self.spin.setValue(0.0)
        self.spin.setFixedWidth(82)

        zero = QtWidgets.QToolButton()
        zero.setIcon(zero_icon)
        zero.setToolTip(f'电机 {index + 1} 归零')
        zero.setFixedSize(26, 26)

        scale = QtWidgets.QWidget()
        scale_layout = QtWidgets.QHBoxLayout(scale)
        scale_layout.setContentsMargins(0, 0, 0, 0)
        scale_layout.setSpacing(0)
        for text, alignment in (
            ('-1.0', QtCore.Qt.AlignLeft),
            ('0', QtCore.Qt.AlignCenter),
            ('+1.0', QtCore.Qt.AlignRight),
        ):
            label = QtWidgets.QLabel(text)
            label.setObjectName('sliderScale')
            label.setAlignment(alignment)
            scale_layout.addWidget(label, 1)

        layout.addWidget(title, 0, 0)
        layout.addWidget(self.slider, 0, 1)
        layout.addWidget(self.spin, 0, 2)
        layout.addWidget(zero, 0, 3)
        layout.addWidget(scale, 1, 1)
        layout.setColumnStretch(1, 1)

        self.slider.valueChanged.connect(self._slider_changed)
        self.spin.valueChanged.connect(self._spin_changed)
        zero.clicked.connect(lambda: self.set_value(0.0))

    def _slider_changed(self, raw):
        value = raw / 1000.0
        blocker = QtCore.QSignalBlocker(self.spin)
        self.spin.setValue(value)
        del blocker
        self.value_changed.emit(value)

    def _spin_changed(self, value):
        blocker = QtCore.QSignalBlocker(self.slider)
        self.slider.setValue(round(value * 1000.0))
        del blocker
        self.value_changed.emit(value)

    def value(self):
        return float(self.spin.value())

    def set_value(self, value):
        self.spin.setValue(float(value))


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, node, store):
        super().__init__()
        self.node = node
        self.store = store
        self._zero_burst_remaining = 0
        self._last_snapshot = None
        self._last_serial_open = None
        self._history_seconds = 30

        self.setWindowTitle('MCU 实时控制台')
        self.resize(1500, 920)
        self.setMinimumSize(1280, 880)
        self.setStyleSheet(self._style_sheet())

        central = QtWidgets.QWidget()
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(16, 12, 16, 12)
        root.setSpacing(10)
        root.addWidget(self._build_header())
        root.addWidget(self._build_status_strip())

        workspace = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        workspace.setChildrenCollapsible(False)
        sensor_panel = self._build_sensor_panel()
        motor_panel = self._build_motor_panel()
        sensor_panel.setMinimumWidth(720)
        motor_panel.setMinimumWidth(400)
        workspace.addWidget(sensor_panel)
        workspace.addWidget(motor_panel)
        workspace.setStretchFactor(0, 1)
        workspace.setStretchFactor(1, 0)
        workspace.setSizes((1020, 440))
        root.addWidget(workspace, 1)
        root.addWidget(self._build_diagnostics_strip())
        self.setCentralWidget(central)

        self.refresh_timer = QtCore.QTimer(self)
        self.refresh_timer.setInterval(100)
        self.refresh_timer.timeout.connect(self._refresh)
        self.refresh_timer.start()

        self.command_timer = QtCore.QTimer(self)
        self.command_timer.setInterval(50)
        self.command_timer.timeout.connect(self._publish_command_tick)
        self.command_timer.start()

    def _build_header(self):
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(widget)
        layout.setContentsMargins(2, 0, 2, 0)

        heading = QtWidgets.QVBoxLayout()
        title = QtWidgets.QLabel('MCU 实时控制台')
        title.setObjectName('windowTitle')
        subtitle = QtWidgets.QLabel(
            f'{self.node.bridge_ns}  ·  IMU / 温压深度 / 8 路电机'
        )
        subtitle.setObjectName('subtitle')
        heading.addWidget(title)
        heading.addWidget(subtitle)
        layout.addLayout(heading)
        layout.addStretch()

        self.pause_plots = QtWidgets.QCheckBox('暂停曲线')
        self.pause_plots.setToolTip('暂停图形刷新；ROS 数据仍继续接收')
        layout.addWidget(self.pause_plots)

        layout.addWidget(QtWidgets.QLabel('窗口'))
        self.history_combo = QtWidgets.QComboBox()
        self.history_combo.addItems(('10 s', '30 s', '60 s'))
        self.history_combo.setCurrentText('30 s')
        self.history_combo.currentTextChanged.connect(self._history_changed)
        layout.addWidget(self.history_combo)

        clear_button = QtWidgets.QToolButton()
        clear_button.setIcon(
            self.style().standardIcon(QtWidgets.QStyle.SP_TrashIcon)
        )
        clear_button.setToolTip('清空历史曲线')
        clear_button.clicked.connect(self.store.clear_history)
        layout.addWidget(clear_button)
        return widget

    def _build_status_strip(self):
        strip = QtWidgets.QFrame()
        strip.setObjectName('statusStrip')
        layout = QtWidgets.QHBoxLayout(strip)
        layout.setContentsMargins(10, 8, 10, 8)
        layout.setSpacing(7)

        self.badges = {
            'serial': StatusBadge('串口 · 等待'),
            'handshake': StatusBadge('握手 · 等待'),
            'uplink': StatusBadge('上行 · 等待'),
            'command': StatusBadge('命令 · 停止'),
            'downlink': StatusBadge('回声 · 等待'),
            'safety': StatusBadge('MCU · 等待'),
        }
        for badge in self.badges.values():
            layout.addWidget(badge)
        layout.addStretch()
        self.status_detail = QtWidgets.QLabel('等待 link_status')
        self.status_detail.setObjectName('statusDetail')
        self.status_detail.setAlignment(
            QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter
        )
        layout.addWidget(self.status_detail, 1)
        return strip

    def _build_sensor_panel(self):
        panel = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(panel)
        layout.setContentsMargins(0, 4, 0, 4)
        layout.setSpacing(10)
        layout.addWidget(self._build_environment_panel())
        layout.addWidget(self._build_imu_panel(), 1)
        return panel

    def _build_imu_panel(self):
        panel = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        heading = QtWidgets.QHBoxLayout()
        title = QtWidgets.QLabel('IMU · 4 组 XYZ')
        title.setObjectName('sectionTitle')
        heading.addWidget(title)
        heading.addStretch()
        self.imu_rate = QtWidgets.QLabel('等待数据')
        self.imu_rate.setObjectName('streamRate')
        heading.addWidget(self.imu_rate)
        layout.addLayout(heading)

        self.imu_table = QtWidgets.QTableWidget(4, 4)
        self.imu_table.setHorizontalHeaderLabels(('数据组', 'X', 'Y', 'Z'))
        self.imu_table.verticalHeader().setVisible(False)
        self.imu_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.imu_table.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self.imu_table.setAlternatingRowColors(True)
        self.imu_table.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        header = self.imu_table.horizontalHeader()
        header.setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeToContents)
        for column in range(1, 4):
            header.setSectionResizeMode(column, QtWidgets.QHeaderView.Stretch)
        for row, name in enumerate(GROUP_NAMES):
            self.imu_table.setItem(row, 0, QtWidgets.QTableWidgetItem(name))
            for column in range(1, 4):
                item = QtWidgets.QTableWidgetItem('--')
                item.setTextAlignment(QtCore.Qt.AlignCenter)
                self.imu_table.setItem(row, column, item)
        self.imu_table.resizeRowsToContents()
        table_height = self.imu_table.horizontalHeader().sizeHint().height()
        table_height += sum(self.imu_table.rowHeight(row) for row in range(4))
        table_height += 2 * self.imu_table.frameWidth() + 2
        self.imu_table.setFixedHeight(table_height)
        layout.addWidget(self.imu_table)

        plots = QtWidgets.QGridLayout()
        plots.setSpacing(8)
        self.imu_plots = []
        self.imu_curves = []
        for group in range(4):
            plot = self._make_plot(GROUP_NAMES[group], '原始值')
            plot.addLegend(offset=(8, 8), labelTextSize='9pt')
            curves = []
            for axis, color in zip(('X', 'Y', 'Z'), AXIS_COLORS):
                curve = plot.plot(name=axis, pen=pg.mkPen(color, width=1.8))
                curve.setClipToView(True)
                curve.setDownsampling(auto=True, method='peak')
                curves.append(curve)
            plots.addWidget(plot, group // 2, group % 2)
            self.imu_plots.append(plot)
            self.imu_curves.append(curves)
        layout.addLayout(plots, 1)
        return panel

    def _build_environment_panel(self):
        panel = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        heading = QtWidgets.QHBoxLayout()
        title = QtWidgets.QLabel('环境数据')
        title.setObjectName('sectionTitle')
        heading.addWidget(title)
        heading.addStretch()
        self.env_rate = QtWidgets.QLabel('等待数据')
        self.env_rate.setObjectName('streamRate')
        heading.addWidget(self.env_rate)
        layout.addLayout(heading)

        body = QtWidgets.QHBoxLayout()
        body.setSpacing(10)
        readouts = QtWidgets.QHBoxLayout()
        readouts.setSpacing(7)
        self.temperature_readout = Readout('温度', '°C', '#b44d12')
        self.pressure_readout = Readout('绝对压力', 'hPa', '#245d9b')
        self.depth_readout = Readout('淡水深度', 'm', '#0a766e')
        readouts.addWidget(self.temperature_readout)
        readouts.addWidget(self.pressure_readout)
        readouts.addWidget(self.depth_readout)
        body.addLayout(readouts, 1)

        calibration = QtWidgets.QFrame()
        calibration.setObjectName('calibration')
        calibration.setFixedWidth(275)
        form = QtWidgets.QGridLayout(calibration)
        form.setContentsMargins(10, 8, 10, 8)
        form.addWidget(QtWidgets.QLabel('水面绝压'), 0, 0)
        self.surface_pressure = QtWidgets.QDoubleSpinBox()
        self.surface_pressure.setRange(500.0, 2000.0)
        self.surface_pressure.setDecimals(2)
        self.surface_pressure.setSingleStep(1.0)
        self.surface_pressure.setValue(1013.25)
        self.surface_pressure.setSuffix(' hPa')
        form.addWidget(self.surface_pressure, 0, 1)

        self.zero_depth_button = QtWidgets.QToolButton()
        self.zero_depth_button.setIcon(
            self.style().standardIcon(QtWidgets.QStyle.SP_DialogApplyButton)
        )
        self.zero_depth_button.setToolTip('以当前压力设置水面零点')
        self.zero_depth_button.setFixedSize(30, 30)
        self.zero_depth_button.setEnabled(False)
        self.zero_depth_button.clicked.connect(self._zero_depth)
        form.addWidget(self.zero_depth_button, 0, 2)

        form.addWidget(QtWidgets.QLabel('淡水密度'), 1, 0)
        self.water_density = QtWidgets.QDoubleSpinBox()
        self.water_density.setRange(900.0, 1100.0)
        self.water_density.setDecimals(1)
        self.water_density.setValue(997.0)
        self.water_density.setSuffix(' kg/m³')
        form.addWidget(self.water_density, 1, 1)
        form.setColumnStretch(1, 1)
        body.addWidget(calibration)
        layout.addLayout(body)
        return panel

    def _build_motor_panel(self):
        panel = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(panel)
        layout.setContentsMargins(10, 0, 0, 0)
        layout.setSpacing(8)

        panel_title = QtWidgets.QLabel('电机测试 · 8 路')
        panel_title.setObjectName('sectionTitle')
        layout.addWidget(panel_title)

        controls = QtWidgets.QFrame()
        controls.setObjectName('motorToolbar')
        toolbar = QtWidgets.QVBoxLayout(controls)
        toolbar.setContentsMargins(10, 8, 10, 8)
        toolbar.setSpacing(6)

        self.arm_checkbox = QtWidgets.QCheckBox('启用 20 Hz 命令发布')
        self.arm_checkbox.setObjectName('armSwitch')
        self.arm_checkbox.toggled.connect(self._armed_toggled)
        toolbar.addWidget(self.arm_checkbox)

        actions = QtWidgets.QHBoxLayout()
        actions.setSpacing(6)

        self.stop_button = QtWidgets.QPushButton('停止并归零')
        self.stop_button.setIcon(
            self.style().standardIcon(QtWidgets.QStyle.SP_MediaStop)
        )
        self.stop_button.setObjectName('stopButton')
        self.stop_button.clicked.connect(self._stop_and_zero)
        actions.addWidget(self.stop_button)

        self.pattern_combo = QtWidgets.QComboBox()
        self.pattern_combo.addItems(('全部为零', '正向阶梯', '正负交替'))
        actions.addWidget(self.pattern_combo, 1)
        load_pattern = QtWidgets.QPushButton('载入')
        load_pattern.clicked.connect(self._load_pattern)
        actions.addWidget(load_pattern)
        toolbar.addLayout(actions)

        self.command_state = QtWidgets.QLabel('未使能，不发布命令')
        self.command_state.setObjectName('commandState')
        self.command_state.setWordWrap(True)
        toolbar.addWidget(self.command_state)
        layout.addWidget(controls)

        channel_container = QtWidgets.QWidget()
        channels_layout = QtWidgets.QVBoxLayout(channel_container)
        channels_layout.setContentsMargins(0, 0, 0, 0)
        channels_layout.setSpacing(5)
        zero_icon = self.style().standardIcon(
            QtWidgets.QStyle.SP_MediaStop
        )
        self.motor_channels = []
        for index in range(8):
            channel = MotorChannel(index, zero_icon)
            channels_layout.addWidget(channel)
            self.motor_channels.append(channel)
        channels_layout.addStretch(1)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        scroll.setWidget(channel_container)
        layout.addWidget(scroll, 1)

        footer = QtWidgets.QFrame()
        footer.setObjectName('motorFooter')
        footer_layout = QtWidgets.QVBoxLayout(footer)
        footer_layout.setContentsMargins(10, 6, 10, 6)
        footer_layout.setSpacing(2)
        self.motor_feedback = QtWidgets.QLabel('等待 MCU 回声')
        self.motor_feedback.setWordWrap(True)
        footer_layout.addWidget(self.motor_feedback)
        range_label = QtWidgets.QLabel('界面输入范围 -1.000 ～ +1.000 · 发布路径不额外限幅')
        range_label.setObjectName('formula')
        range_label.setWordWrap(True)
        footer_layout.addWidget(range_label)
        layout.addWidget(footer)
        return panel

    def _build_diagnostics_strip(self):
        strip = QtWidgets.QFrame()
        strip.setObjectName('diagnosticsStrip')
        layout = QtWidgets.QHBoxLayout(strip)
        layout.setContentsMargins(10, 6, 10, 6)
        title = QtWidgets.QLabel('链路诊断')
        title.setObjectName('diagnosticsTitle')
        layout.addWidget(title)
        self.diagnostics_label = QtWidgets.QLabel('等待 /mcu_bridge/diagnostics')
        self.diagnostics_label.setObjectName('diagnosticsText')
        layout.addWidget(self.diagnostics_label, 1)
        return strip

    def _make_plot(self, title, unit):
        plot = pg.PlotWidget(background='#ffffff')
        plot.setMinimumHeight(135)
        plot.setTitle(title, color='#27333b', size='10pt')
        plot.showGrid(x=True, y=True, alpha=0.18)
        plot.setLabel('left', unit)
        plot.setLabel('bottom', '最近时间', units='s')
        plot.getAxis('left').setTextPen('#53616b')
        plot.getAxis('bottom').setTextPen('#53616b')
        plot.setMouseEnabled(x=True, y=True)
        return plot

    def _history_changed(self, text):
        self._history_seconds = int(text.split()[0])

    def _zero_depth(self):
        snapshot = self.store.snapshot()
        if snapshot.pressures:
            self.surface_pressure.setValue(snapshot.pressures[-1] / 100.0)

    def _armed_toggled(self, armed):
        if armed:
            snapshot = self.store.snapshot()
            link = snapshot.link
            if link is not None and (not link.serial_open or link.version_mismatch):
                self.arm_checkbox.setChecked(False)
                reason = '串口未打开' if not link.serial_open else '协议版本不匹配'
                self.command_state.setText(f'无法使能：{reason}')
                return
            self._zero_burst_remaining = 0
            self.command_state.setText('正在按 20 Hz 发布')
            self.arm_checkbox.setStyleSheet('color: #12624d; font-weight: 700;')
        else:
            self._zero_burst_remaining = 4
            self.command_state.setText('正在发送归零帧，随后停发')
            self.arm_checkbox.setStyleSheet('')

    def _stop_and_zero(self):
        for channel in self.motor_channels:
            channel.set_value(0.0)
        if self.arm_checkbox.isChecked():
            self.arm_checkbox.setChecked(False)
        else:
            self._zero_burst_remaining = 4
            self.command_state.setText('正在发送归零帧，随后停发')

    def _load_pattern(self):
        name = self.pattern_combo.currentText()
        if name == '正向阶梯':
            values = tuple((index + 1) / 8.0 for index in range(8))
        elif name == '正负交替':
            values = (1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0)
        else:
            values = (0.0,) * 8
        for channel, value in zip(self.motor_channels, values):
            channel.set_value(value)

    def _motor_values(self):
        return tuple(channel.value() for channel in self.motor_channels)

    def _publish_command_tick(self):
        try:
            if self.arm_checkbox.isChecked():
                self.node.publish_command(self._motor_values())
            elif self._zero_burst_remaining > 0:
                self.node.publish_command((0.0,) * 8)
                self._zero_burst_remaining -= 1
                if self._zero_burst_remaining == 0:
                    self.command_state.setText('未使能，不发布命令')
        except (RuntimeError, ValueError) as error:
            self.arm_checkbox.setChecked(False)
            self.command_state.setText(f'发布失败：{error}')

    def _refresh(self):
        snapshot = self.store.snapshot()
        self._last_snapshot = snapshot
        self._refresh_status(snapshot)
        self._refresh_readouts(snapshot)
        self._refresh_diagnostics(snapshot)
        if not self.pause_plots.isChecked():
            self._refresh_plots(snapshot)

    @staticmethod
    def _format_rate(metrics, expected):
        if metrics.count == 0:
            return '等待数据'
        if metrics.stale:
            return f'已停止 · {metrics.age_s:.1f} s 无新帧'
        rate = '--' if metrics.hz is None else f'{metrics.hz:.1f}'
        return f'{rate} Hz / 期望 {expected} · 空洞 {metrics.gaps}'

    def _refresh_status(self, snapshot):
        link = snapshot.link
        uplink_ok = snapshot.imu_metrics.count > 0 and not snapshot.imu_metrics.stale
        self.badges['uplink'].set_state(
            '上行 · 正常' if uplink_ok else '上行 · 等待' if snapshot.imu_metrics.count == 0 else '上行 · 中断',
            'ok' if uplink_ok else 'idle' if snapshot.imu_metrics.count == 0 else 'error',
        )
        self.imu_rate.setText(self._format_rate(snapshot.imu_metrics, 20))
        self.env_rate.setText(self._format_rate(snapshot.env_metrics, 20))
        self.imu_rate.setProperty('stale', snapshot.imu_metrics.stale)
        self.env_rate.setProperty('stale', snapshot.env_metrics.stale)
        self.imu_rate.style().unpolish(self.imu_rate)
        self.imu_rate.style().polish(self.imu_rate)
        self.env_rate.style().unpolish(self.env_rate)
        self.env_rate.style().polish(self.env_rate)

        if link is None:
            return

        self.badges['serial'].set_state(
            '串口 · 打开' if link.serial_open else '串口 · 断开',
            'ok' if link.serial_open else 'error',
        )
        self.badges['handshake'].set_state(
            '握手 · 完成' if link.handshaked else '握手 · 等待',
            'ok' if link.handshaked else 'warn',
        )
        self.badges['command'].set_state(
            '命令 · 流动' if link.commands_flowing else '命令 · 停止',
            'ok' if link.commands_flowing else 'idle',
        )
        if link.downlink_healthy:
            self.badges['downlink'].set_state('回声 · 推进', 'ok')
        elif link.commands_flowing:
            self.badges['downlink'].set_state('回声 · 未确认', 'error')
        else:
            self.badges['downlink'].set_state('回声 · 等待', 'idle')

        if link.version_mismatch:
            self.badges['safety'].set_state('MCU · 版本错误', 'error')
        elif link.safe_state:
            self.badges['safety'].set_state('MCU · 安全态', 'warn')
        elif link.cmd_stale:
            self.badges['safety'].set_state('MCU · 命令过期', 'warn')
        else:
            self.badges['safety'].set_state('MCU · 正常', 'ok')

        self.status_detail.setText(
            f'状态 0x{link.status:02X}   上行空洞 {link.uplink_gaps}   '
            f'CRC {link.crc_errors}   重同步 {link.resyncs}   重连 {link.reconnects}'
        )

        if self.arm_checkbox.isChecked() and (
            not link.serial_open or link.version_mismatch
        ):
            self.arm_checkbox.setChecked(False)

        self._last_serial_open = link.serial_open

    def _refresh_readouts(self, snapshot):
        if snapshot.imu_values:
            latest = snapshot.imu_values[-1]
            for group in range(4):
                for axis in range(3):
                    self.imu_table.item(group, axis + 1).setText(
                        f'{latest[group * 3 + axis]:.5f}'
                    )

        if not snapshot.pressures:
            return
        temperature = snapshot.temperatures[-1]
        pressure = snapshot.pressures[-1]
        depth = depth_from_pressure(
            pressure, self.surface_pressure.value() * 100.0, self.water_density.value()
        )
        self.temperature_readout.set_value(temperature, 2)
        self.pressure_readout.set_value(pressure / 100.0, 2)
        self.depth_readout.set_value(depth, 3)
        self.zero_depth_button.setEnabled(True)

    def _refresh_plots(self, snapshot):
        if snapshot.imu_times:
            times = np.asarray(snapshot.imu_times, dtype=float)
            times -= times[-1]
            values = np.asarray(snapshot.imu_values, dtype=float)
            for group, (plot, curves) in enumerate(
                zip(self.imu_plots, self.imu_curves)
            ):
                for axis, curve in enumerate(curves):
                    curve.setData(times, values[:, group * 3 + axis])
                plot.setXRange(-self._history_seconds, 0.0, padding=0.0)

    def _refresh_diagnostics(self, snapshot):
        diag = snapshot.diagnostics
        if diag is None:
            self.motor_feedback.setText(
                f'本地已发布 {snapshot.commands_published} 帧 · 等待 MCU 诊断回声'
            )
            return
        self.diagnostics_label.setText(
            f'MCU 收下行 {diag.rx_frames_ok}   CRC 错 {diag.rx_crc_err}   '
            f'重同步 {diag.rx_resync}   ORE {diag.rx_overrun}   '
            f'DMA 丢 {diag.rx_dma_lost}   TX 丢 {diag.tx_dropped}   '
            f'最近命令 {diag.last_cmd_age_ms} ms'
        )
        echo = '--' if snapshot.last_cmd_seq is None else str(snapshot.last_cmd_seq)
        healthy = snapshot.link is not None and snapshot.link.downlink_healthy
        confirmation = '回声正在推进' if healthy else '尚未确认推进'
        self.motor_feedback.setText(
            f'本地已发布 {snapshot.commands_published} 帧 · MCU last_cmd_seq {echo} · {confirmation}'
        )

    def closeEvent(self, event):
        if self.arm_checkbox.isChecked() or self._zero_burst_remaining > 0:
            try:
                for _ in range(3):
                    self.node.publish_command((0.0,) * 8)
            except (RuntimeError, ValueError):
                pass
        event.accept()

    @staticmethod
    def _style_sheet():
        return """
            QMainWindow, QWidget {
                background: #f4f6f7;
                color: #25313a;
                font-family: "Noto Sans CJK SC", "Microsoft YaHei", sans-serif;
                font-size: 12px;
            }
            QLabel#windowTitle { font-size: 23px; font-weight: 700; color: #17232b; }
            QLabel#subtitle { color: #687781; }
            QLabel#sectionTitle { font-size: 16px; font-weight: 650; color: #1e2b33; }
            QLabel#streamRate { color: #53616b; }
            QLabel#streamRate[stale="true"] { color: #a22d38; font-weight: 650; }
            QFrame#statusStrip, QFrame#diagnosticsStrip, QFrame#motorToolbar,
            QFrame#motorFooter, QFrame#calibration {
                background: #ffffff;
                border: 1px solid #d9dfe3;
                border-radius: 6px;
            }
            QLabel#statusDetail, QLabel#diagnosticsText { color: #53616b; }
            QLabel#diagnosticsTitle { font-weight: 700; color: #27343c; }
            QFrame#readout, QFrame#motorChannel {
                background: #ffffff;
                border: 1px solid #d9dfe3;
                border-radius: 6px;
            }
            QLabel#readoutTitle { color: #5a6872; font-weight: 600; }
            QLabel#sliderScale, QLabel#formula { color: #76838c; }
            QLabel#motorTitle { font-size: 14px; font-weight: 650; min-width: 48px; }
            QLabel#commandState { color: #53616b; font-weight: 600; }
            QPushButton, QToolButton, QComboBox, QDoubleSpinBox {
                background: #ffffff;
                border: 1px solid #bdc7cd;
                border-radius: 4px;
                padding: 5px 8px;
                min-height: 19px;
            }
            QPushButton:hover, QToolButton:hover { background: #edf4f6; border-color: #79939f; }
            QPushButton:pressed, QToolButton:pressed { background: #dce8eb; }
            QPushButton#stopButton { color: #922b35; border-color: #d9a0a5; font-weight: 650; }
            QCheckBox { spacing: 7px; }
            QCheckBox#armSwitch { font-size: 14px; font-weight: 650; }
            QTableWidget {
                background: #ffffff; alternate-background-color: #f7f9fa;
                border: 1px solid #d9dfe3; gridline-color: #e1e6e9;
            }
            QHeaderView::section {
                background: #edf1f3; color: #45545d; padding: 5px;
                border: none; border-right: 1px solid #d8dfe3;
            }
            QSlider::groove:horizontal { height: 5px; background: #d6dde1; border-radius: 2px; }
            QSlider::sub-page:horizontal { background: #378578; border-radius: 2px; }
            QSlider::handle:horizontal {
                background: #ffffff; border: 2px solid #287065; width: 15px;
                margin: -6px 0; border-radius: 7px;
            }
            QSplitter::handle { background: #d8dfe3; width: 1px; }
        """

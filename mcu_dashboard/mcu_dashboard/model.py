"""Thread-safe telemetry history and unit conversion helpers."""

from collections import deque
from dataclasses import dataclass
import math
import threading
import time
from typing import Optional, Sequence, Tuple


SEQ_MOD = 1 << 16
GRAVITY_M_S2 = 9.80665


def depth_from_pressure(
    pressure_pa: float,
    surface_pressure_pa: float,
    density_kg_m3: float = 997.0,
) -> float:
    """Return freshwater depth from absolute pressure using a surface reference."""
    values = (pressure_pa, surface_pressure_pa, density_kg_m3)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('pressure and density must be finite')
    if density_kg_m3 <= 0.0:
        raise ValueError('density must be positive')
    return (pressure_pa - surface_pressure_pa) / (density_kg_m3 * GRAVITY_M_S2)


@dataclass(frozen=True)
class StreamMetrics:
    count: int = 0
    hz: Optional[float] = None
    gaps: int = 0
    duplicates: int = 0
    age_s: Optional[float] = None
    stale: bool = False


@dataclass(frozen=True)
class LinkData:
    status: int
    safe_state: bool
    cmd_stale: bool
    not_handshaked: bool
    version_mismatch: bool
    serial_open: bool
    handshaked: bool
    downlink_healthy: bool
    commands_flowing: bool
    uplink_gaps: int
    crc_errors: int
    resyncs: int
    reconnects: int


@dataclass(frozen=True)
class DiagnosticsData:
    rx_frames_ok: int
    rx_crc_err: int
    rx_resync: int
    rx_overrun: int
    rx_dma_lost: int
    tx_dropped: int
    last_cmd_age_ms: int


@dataclass(frozen=True)
class DashboardSnapshot:
    imu_times: Tuple[float, ...]
    imu_values: Tuple[Tuple[float, ...], ...]
    env_times: Tuple[float, ...]
    temperatures: Tuple[float, ...]
    pressures: Tuple[float, ...]
    imu_metrics: StreamMetrics
    env_metrics: StreamMetrics
    link_metrics: StreamMetrics
    diagnostics_metrics: StreamMetrics
    link: Optional[LinkData]
    diagnostics: Optional[DiagnosticsData]
    last_cmd_seq: Optional[int]
    commands_published: int


class _StreamTracker:
    def __init__(self, expected_hz: float, rate_window: int = 40):
        self.expected_hz = expected_hz
        self.arrivals = deque(maxlen=rate_window)
        self.count = 0
        self.gaps = 0
        self.duplicates = 0
        self.previous_seq = None

    def update(self, now: float, seq: Optional[int] = None) -> None:
        self.arrivals.append(now)
        self.count += 1
        if seq is None:
            return
        if self.previous_seq is not None:
            gap = (int(seq) - self.previous_seq) % SEQ_MOD
            if gap == 0:
                self.duplicates += 1
            elif gap > 1:
                self.gaps += gap - 1
        self.previous_seq = int(seq)

    def snapshot(self, now: float) -> StreamMetrics:
        hz = None
        if len(self.arrivals) >= 5:
            span = self.arrivals[-1] - self.arrivals[0]
            if span > 0.0:
                hz = (len(self.arrivals) - 1) / span
        age = None if not self.arrivals else now - self.arrivals[-1]
        stale_after = max(0.5, 3.0 / self.expected_hz) if self.expected_hz else 3.0
        return StreamMetrics(
            count=self.count,
            hz=hz,
            gaps=self.gaps,
            duplicates=self.duplicates,
            age_s=age,
            stale=age is not None and age > stale_after,
        )


class TelemetryStore:
    """ROS callbacks write here; the Qt thread consumes immutable snapshots."""

    def __init__(self, history_seconds: float = 60.0, expected_hz: float = 20.0):
        max_samples = max(100, int(history_seconds * expected_hz))
        self._lock = threading.Lock()
        self._started = time.monotonic()
        self._imu_times = deque(maxlen=max_samples)
        self._imu_values = deque(maxlen=max_samples)
        self._env_times = deque(maxlen=max_samples)
        self._temperatures = deque(maxlen=max_samples)
        self._pressures = deque(maxlen=max_samples)
        self._imu_tracker = _StreamTracker(expected_hz)
        self._env_tracker = _StreamTracker(expected_hz)
        self._link_tracker = _StreamTracker(1.0)
        self._diag_tracker = _StreamTracker(1.0)
        self._link = None
        self._diagnostics = None
        self._last_cmd_seq = None
        self._commands_published = 0

    def update_imu(self, seq: int, last_cmd_seq: int, values: Sequence[float]) -> None:
        if len(values) != 16:
            return
        now = time.monotonic()
        sample = tuple(float(value) for value in values)
        with self._lock:
            self._imu_tracker.update(now, seq)
            self._imu_times.append(now - self._started)
            self._imu_values.append(sample)
            self._last_cmd_seq = int(last_cmd_seq)

    def update_env(
        self, seq: int, last_cmd_seq: int, temperature: float, pressure: float
    ) -> None:
        now = time.monotonic()
        with self._lock:
            self._env_tracker.update(now, seq)
            self._env_times.append(now - self._started)
            self._temperatures.append(float(temperature))
            self._pressures.append(float(pressure))
            self._last_cmd_seq = int(last_cmd_seq)

    def update_link(self, msg) -> None:
        now = time.monotonic()
        data = LinkData(
            status=int(msg.status),
            safe_state=bool(msg.safe_state),
            cmd_stale=bool(msg.cmd_stale),
            not_handshaked=bool(msg.not_handshaked),
            version_mismatch=bool(msg.version_mismatch),
            serial_open=bool(msg.serial_open),
            handshaked=bool(msg.handshaked),
            downlink_healthy=bool(msg.downlink_healthy),
            commands_flowing=bool(msg.commands_flowing),
            uplink_gaps=int(msg.uplink_gaps),
            crc_errors=int(msg.crc_errors),
            resyncs=int(msg.resyncs),
            reconnects=int(msg.reconnects),
        )
        with self._lock:
            self._link_tracker.update(now)
            self._link = data

    def update_diagnostics(self, msg) -> None:
        now = time.monotonic()
        data = DiagnosticsData(
            rx_frames_ok=int(msg.rx_frames_ok),
            rx_crc_err=int(msg.rx_crc_err),
            rx_resync=int(msg.rx_resync),
            rx_overrun=int(msg.rx_overrun),
            rx_dma_lost=int(msg.rx_dma_lost),
            tx_dropped=int(msg.tx_dropped),
            last_cmd_age_ms=int(msg.last_cmd_age_ms),
        )
        with self._lock:
            self._diag_tracker.update(now, int(msg.seq))
            self._diagnostics = data
            self._last_cmd_seq = int(msg.last_cmd_seq)

    def note_command_published(self) -> None:
        with self._lock:
            self._commands_published += 1

    def clear_history(self) -> None:
        with self._lock:
            self._started = time.monotonic()
            self._imu_times.clear()
            self._imu_values.clear()
            self._env_times.clear()
            self._temperatures.clear()
            self._pressures.clear()

    def snapshot(self) -> DashboardSnapshot:
        now = time.monotonic()
        with self._lock:
            return DashboardSnapshot(
                imu_times=tuple(self._imu_times),
                imu_values=tuple(self._imu_values),
                env_times=tuple(self._env_times),
                temperatures=tuple(self._temperatures),
                pressures=tuple(self._pressures),
                imu_metrics=self._imu_tracker.snapshot(now),
                env_metrics=self._env_tracker.snapshot(now),
                link_metrics=self._link_tracker.snapshot(now),
                diagnostics_metrics=self._diag_tracker.snapshot(now),
                link=self._link,
                diagnostics=self._diagnostics,
                last_cmd_seq=self._last_cmd_seq,
                commands_published=self._commands_published,
            )

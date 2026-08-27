import math

import pytest

from mcu_dashboard.model import TelemetryStore, depth_from_pressure


def test_depth_is_zero_at_surface_reference():
    assert depth_from_pressure(101325.0, 101325.0) == pytest.approx(0.0)


def test_freshwater_depth_from_pressure_difference():
    pressure = 101325.0 + 997.0 * 9.80665 * 2.5
    assert depth_from_pressure(pressure, 101325.0, 997.0) == pytest.approx(2.5)


@pytest.mark.parametrize(
    'pressure, reference, density',
    [
        (math.nan, 101325.0, 997.0),
        (101325.0, math.inf, 997.0),
        (101325.0, 101325.0, 0.0),
        (101325.0, 101325.0, -1.0),
    ],
)
def test_depth_rejects_invalid_inputs(pressure, reference, density):
    with pytest.raises(ValueError):
        depth_from_pressure(pressure, reference, density)


def test_store_groups_sixteen_imu_values_and_tracks_seq_gaps():
    store = TelemetryStore(history_seconds=1.0, expected_hz=20.0)
    store.update_imu(10, 3, tuple(range(16)))
    store.update_imu(12, 4, tuple(range(16, 32)))

    snapshot = store.snapshot()
    assert snapshot.imu_values[-1] == tuple(float(value) for value in range(16, 32))
    assert snapshot.imu_metrics.count == 2
    assert snapshot.imu_metrics.gaps == 1
    assert snapshot.last_cmd_seq == 4


def test_store_handles_uint16_sequence_wrap_without_gap():
    store = TelemetryStore()
    store.update_env(65535, 0, 25.0, 101325.0)
    store.update_env(0, 0, 25.1, 101326.0)

    assert store.snapshot().env_metrics.gaps == 0


def test_clear_history_keeps_latest_status_and_counters():
    store = TelemetryStore()
    store.update_imu(1, 0, (0.0,) * 16)
    store.note_command_published()
    store.clear_history()

    snapshot = store.snapshot()
    assert snapshot.imu_times == ()
    assert snapshot.imu_values == ()
    assert snapshot.imu_metrics.count == 1
    assert snapshot.commands_published == 1

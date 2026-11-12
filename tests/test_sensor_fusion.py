import numpy as np
import pytest
from sensor_fusion import SensorFusionEKF, SensorReading

G = 9.80665


def _imu(t_us: int, accel=(0.0, 0.0, -G), gyro=(0.0, 0.0, 0.0)) -> SensorReading:
    return SensorReading(timestamp_us=t_us, accel_xyz=np.array(accel), gyro_xyz=np.array(gyro))


def test_stationary_imu_stays_at_origin():
    # A level accelerometer at rest reads -g on body z (z-down). The filter must not drift.
    ekf = SensorFusionEKF()
    for k in range(101):
        ekf.predict(_imu(k * 10_000))
    assert np.allclose(ekf.state.position_ned, 0.0, atol=1e-9)
    assert np.allclose(ekf.state.velocity_ned, 0.0, atol=1e-9)


def test_constant_north_acceleration_integrates():
    ekf = SensorFusionEKF()
    for k in range(101):  # 1 s at 100 Hz, 2 m/s^2 north
        ekf.predict(_imu(k * 10_000, accel=(2.0, 0.0, -G)))
    assert ekf.state.velocity_ned[0] == pytest.approx(2.0, rel=1e-6)
    assert ekf.state.position_ned[0] == pytest.approx(1.0, rel=1e-6)


def test_quaternion_stays_unit_norm_while_rotating():
    ekf = SensorFusionEKF()
    for k in range(201):
        ekf.predict(_imu(k * 10_000, gyro=(0.3, -0.2, 0.5)))
    assert np.linalg.norm(ekf.state.attitude_quat) == pytest.approx(1.0, abs=1e-12)


def test_gps_update_pulls_position_and_shrinks_covariance():
    ekf = SensorFusionEKF()
    p_before = np.trace(ekf.state.covariance[:3, :3])
    # 10 m north of origin in the module's linearised LLA convention.
    reading = _imu(0)
    reading.gps_lla = np.array([10.0 / 111319.0, 0.0, 0.0])
    reading.gps_quality = 2
    ekf.update_gps(reading)
    assert 0.0 < ekf.state.position_ned[0] < 10.0
    assert np.trace(ekf.state.covariance[:3, :3]) < p_before


def test_gps_without_3d_fix_is_ignored():
    ekf = SensorFusionEKF()
    reading = _imu(0)
    reading.gps_lla = np.array([1e-3, 0.0, 0.0])
    reading.gps_quality = 1
    ekf.update_gps(reading)
    assert np.allclose(ekf.state.position_ned, 0.0)

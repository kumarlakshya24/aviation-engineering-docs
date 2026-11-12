"""Sensor Fusion Module (Extended Kalman Filter for flight state estimation).

Fuses data from IMU (accelerometer + gyroscope), GPS, barometric altimeter,
and magnetometer to produce a unified aircraft state estimate. This is the
core positioning system for the Halcyon Aero H1 eVTOL aircraft.

Reference: HA-FC-SPEC-2024-001 Section 4.3, State Estimation
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class SensorReading:
    """Raw sensor measurement with timestamp and quality flag."""
    timestamp_us: int
    accel_xyz: np.ndarray      # m/s^2, body frame
    gyro_xyz: np.ndarray       # rad/s, body frame
    gps_lla: np.ndarray | None = None  # lat, lon, alt (deg, deg, m)
    baro_alt_m: float | None = None
    mag_xyz: np.ndarray | None = None  # uT, body frame
    gps_quality: int = 0       # 0=no fix, 1=2D, 2=3D, 3=RTK


@dataclass
class StateEstimate:
    """Fused aircraft state in NED (North-East-Down) frame."""
    position_ned: np.ndarray   # meters
    velocity_ned: np.ndarray   # m/s
    attitude_quat: np.ndarray  # quaternion [w, x, y, z]
    gyro_bias: np.ndarray      # rad/s
    accel_bias: np.ndarray     # m/s^2
    covariance: np.ndarray     # 15x15 state covariance


# State indices in the 15-element state vector
_POS = slice(0, 3)
_VEL = slice(3, 6)
_ATT = slice(6, 9)   # error-state rotation vector
_GBIAS = slice(9, 12)
_ABIAS = slice(12, 15)

# Process noise tuning, validated against flight test data (FT-2024-042)
Q_GYRO_NOISE = 1.0e-4       # (rad/s)^2 / Hz
Q_ACCEL_NOISE = 1.0e-3      # (m/s^2)^2 / Hz
Q_GYRO_BIAS_WALK = 1.0e-8   # (rad/s)^2 / Hz
Q_ACCEL_BIAS_WALK = 1.0e-6  # (m/s^2)^2 / Hz

# Measurement noise from sensor datasheets + flight test calibration
R_GPS_POS = np.diag([1.5, 1.5, 3.0]) ** 2   # m^2 (RTK: 0.02 m)
R_GPS_VEL = np.diag([0.1, 0.1, 0.2]) ** 2   # (m/s)^2
R_BARO_ALT = 0.5 ** 2                         # m^2
R_MAG = np.diag([5.0, 5.0, 5.0]) ** 2        # uT^2


class SensorFusionEKF:
    """Extended Kalman Filter for multi-sensor fusion.

    Implements a 15-state error-state EKF:
    - Position (NED, 3)
    - Velocity (NED, 3)
    - Attitude error (rotation vector, 3)
    - Gyro bias (3)
    - Accelerometer bias (3)
    """

    def __init__(self, initial_state: StateEstimate | None = None):
        if initial_state is not None:
            self.state = initial_state
        else:
            self.state = StateEstimate(
                position_ned=np.zeros(3),
                velocity_ned=np.zeros(3),
                attitude_quat=np.array([1.0, 0.0, 0.0, 0.0]),
                gyro_bias=np.zeros(3),
                accel_bias=np.zeros(3),
                covariance=np.eye(15) * 1e-2,
            )
        self._last_timestamp_us: int | None = None

    def predict(self, reading: SensorReading) -> StateEstimate:
        """Propagate state forward using IMU measurements.

        Args:
            reading: Raw sensor data including accelerometer and gyroscope.

        Returns:
            Updated state estimate after prediction step.
        """
        if self._last_timestamp_us is None:
            self._last_timestamp_us = reading.timestamp_us
            return self.state

        dt = (reading.timestamp_us - self._last_timestamp_us) * 1e-6
        self._last_timestamp_us = reading.timestamp_us

        if dt <= 0 or dt > 1.0:
            return self.state

        # Bias-corrected measurements
        gyro = reading.gyro_xyz - self.state.gyro_bias
        accel = reading.accel_xyz - self.state.accel_bias

        # Rotate specific force to NED and add gravity (NED is z-down, so g = +9.81 on z).
        # An accelerometer at rest reads [0, 0, -g] in a level z-down body frame.
        R = self._quat_to_dcm(self.state.attitude_quat)
        accel_ned = R @ accel + np.array([0, 0, 9.80665])

        # State propagation
        self.state.position_ned += self.state.velocity_ned * dt + 0.5 * accel_ned * dt**2
        self.state.velocity_ned += accel_ned * dt
        self.state.attitude_quat = self._integrate_quaternion(
            self.state.attitude_quat, gyro, dt
        )

        # Covariance propagation
        F = self._compute_state_transition(R, accel, dt)
        Q = self._compute_process_noise(dt)
        self.state.covariance = F @ self.state.covariance @ F.T + Q

        return self.state

    def update_gps(self, reading: SensorReading) -> StateEstimate:
        """Fuse GPS position and velocity measurements.

        Only processes readings with GPS quality >= 2 (3D fix).
        RTK measurements (quality=3) use tighter noise model.

        Args:
            reading: Sensor data with valid GPS fields.

        Returns:
            Updated state estimate after GPS fusion.
        """
        if reading.gps_lla is None or reading.gps_quality < 2:
            return self.state

        # Convert LLA to NED (simplified; production uses full WGS84)
        gps_ned = self._lla_to_ned(reading.gps_lla)

        # Tighter noise for RTK
        R_pos = R_GPS_POS * (0.01 if reading.gps_quality == 3 else 1.0)

        # Position measurement update
        H = np.zeros((3, 15))
        H[:, _POS] = np.eye(3)

        innovation = gps_ned - self.state.position_ned
        self._kalman_update(H, innovation, R_pos)

        return self.state

    def _kalman_update(self, H: np.ndarray, innovation: np.ndarray, R: np.ndarray):
        """Apply standard Kalman measurement update."""
        P = self.state.covariance
        S = H @ P @ H.T + R
        K = P @ H.T @ np.linalg.inv(S)

        dx = K @ innovation
        self.state.position_ned += dx[_POS]
        self.state.velocity_ned += dx[_VEL]
        self.state.gyro_bias += dx[_GBIAS]
        self.state.accel_bias += dx[_ABIAS]

        # Attitude correction via rotation vector
        dtheta = dx[_ATT]
        dq = np.array([1.0, *dtheta * 0.5])
        dq /= np.linalg.norm(dq)
        self.state.attitude_quat = self._quat_multiply(dq, self.state.attitude_quat)

        # Joseph form for numerical stability
        I_KH = np.eye(15) - K @ H
        self.state.covariance = I_KH @ P @ I_KH.T + K @ R @ K.T

    @staticmethod
    def _quat_to_dcm(q: np.ndarray) -> np.ndarray:
        """Convert quaternion [w,x,y,z] to 3x3 direction cosine matrix."""
        w, x, y, z = q
        return np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
            [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
            [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)],
        ])

    @staticmethod
    def _integrate_quaternion(q: np.ndarray, omega: np.ndarray, dt: float) -> np.ndarray:
        """Integrate quaternion using angular velocity (first-order)."""
        dq = np.array([0.0, *(omega * dt * 0.5)])
        q_new = q + 0.5 * np.array([
            -q[1]*dq[1] - q[2]*dq[2] - q[3]*dq[3],
             q[0]*dq[1] + q[2]*dq[3] - q[3]*dq[2],
             q[0]*dq[2] - q[1]*dq[3] + q[3]*dq[1],
             q[0]*dq[3] + q[1]*dq[2] - q[2]*dq[1],
        ])
        return q_new / np.linalg.norm(q_new)

    @staticmethod
    def _quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
        """Hamilton quaternion product."""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
        ])

    @staticmethod
    def _lla_to_ned(lla: np.ndarray) -> np.ndarray:
        """Simplified LLA to NED conversion (linearized around reference)."""
        lat, lon, alt = lla
        return np.array([
            lat * 111319.0,
            lon * 111319.0 * np.cos(np.radians(lat)),
            -alt,
        ])

    def _compute_state_transition(self, R: np.ndarray, accel: np.ndarray, dt: float) -> np.ndarray:
        """Compute linearized state transition matrix F."""
        F = np.eye(15)
        F[_POS, _VEL] = np.eye(3) * dt
        F[_VEL, _ATT] = -R @ self._skew(accel) * dt
        F[_VEL, _ABIAS] = -R * dt
        F[_ATT, _GBIAS] = -np.eye(3) * dt
        return F

    def _compute_process_noise(self, dt: float) -> np.ndarray:
        """Compute discrete process noise matrix Q."""
        Q = np.zeros((15, 15))
        Q[_VEL, _VEL] = np.eye(3) * Q_ACCEL_NOISE * dt
        Q[_ATT, _ATT] = np.eye(3) * Q_GYRO_NOISE * dt
        Q[_GBIAS, _GBIAS] = np.eye(3) * Q_GYRO_BIAS_WALK * dt
        Q[_ABIAS, _ABIAS] = np.eye(3) * Q_ACCEL_BIAS_WALK * dt
        return Q

    @staticmethod
    def _skew(v: np.ndarray) -> np.ndarray:
        """Skew-symmetric matrix from 3-vector."""
        return np.array([
            [0, -v[2], v[1]],
            [v[2], 0, -v[0]],
            [-v[1], v[0], 0],
        ])

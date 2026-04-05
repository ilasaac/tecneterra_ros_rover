"""
diff_drive.py — Differential-drive physics model for simulation.

Pure Python, no ROS2 dependency.  Used by sim_harness_node (on-Jetson ROS2 sim)
and by tools/sim_navigator.py (off-Jetson SIL).

Ported from tools/sim_navigator.py DiffDriveState and tools/simulator.py RoverState.
"""

from __future__ import annotations

import math


# ── Default physical parameters (tuned from real rover dynamics data) ────────

DEFAULT_TTR_PHYS: dict = {
    'track_width_m':              0.9,
    'max_wheel_mms':              1650,
    'steering_scale':             0.37,
    'accel_cap_mms_per_tick':     7,
    'decel_divisor':              1,
    'angular_diff_limit_mms':     275,
    'smooth_tick_hz':             100,
    'rotation_center_offset_m':   0.35,
    'steer_lag_s':                0.25,
}


class RoverState:
    """Simple skid-steer dead-reckoning model (base class)."""

    def __init__(self, lat: float, lon: float, heading_deg: float = 0.0):
        self.lat = lat
        self.lon = lon
        self.heading_rad = math.radians(heading_deg)
        self.speed_mps = 0.0

    def update(self, throttle: int, steering: int,
               max_speed: float, wheelbase: float, dt: float,
               turn_scale: float = 0.1, steer_deadband: float = 0.05):
        speed = (throttle - 1500) / 500.0 * max_speed
        steer = (steering - 1500) / 500.0
        if abs(steer) < steer_deadband:
            steer = 0.0
        omega = -(turn_scale * 2.0 * steer * max_speed / wheelbase) if wheelbase > 0 else 0.0
        self.heading_rad += omega * dt
        lat_rad = math.radians(self.lat)
        cos_lat = math.cos(lat_rad) or 1e-9
        self.lat += (speed * math.cos(self.heading_rad) * dt) / 111320.0
        self.lon += (speed * math.sin(self.heading_rad) * dt) / (111320.0 * cos_lat)
        self.speed_mps = speed

    def secondary_pos(self, baseline_m: float):
        """Position of front antenna (baseline_m ahead along heading)."""
        lat_rad = math.radians(self.lat)
        cos_lat = math.cos(lat_rad) or 1e-9
        s_lat = self.lat + (baseline_m * math.cos(self.heading_rad)) / 111320.0
        s_lon = self.lon + (baseline_m * math.sin(self.heading_rad)) / (111320.0 * cos_lat)
        return s_lat, s_lon

    @property
    def heading_deg(self):
        return math.degrees(self.heading_rad) % 360.0


class DiffDriveState(RoverState):
    """Differential-drive rover model tuned from real rover dynamics data.

    Converts PPM throttle+steer to left/right wheel speeds (mm/s), applies
    steering_scale (real motor authority), angular velocity limit, and
    integrates position via the track-width kinematic model.
    """

    def __init__(self, lat: float, lon: float, heading_deg: float = 0.0,
                 ttr_phys: dict | None = None, max_steer: float = 0.8):
        super().__init__(lat, lon, heading_deg)
        p = {**DEFAULT_TTR_PHYS, **(ttr_phys or {})}
        self._track_m = p['track_width_m']
        self._max_wheel = float(p['max_wheel_mms'])
        self._steer_scale = float(p.get('steering_scale', 0.37))
        self._accel_cap = float(p['accel_cap_mms_per_tick'])
        self._decel_div = float(p['decel_divisor'])
        self._ang_diff_lim = float(p['angular_diff_limit_mms'])
        self._smooth_hz = float(p['smooth_tick_hz'])
        self._rc_offset = float(p.get('rotation_center_offset_m', 0.0))
        self._max_steer = max_steer
        self._last_left = 0.0
        self._last_right = 0.0
        self._steer_lag = float(p.get('steer_lag_s', 0.0))
        self._steer_buf: list[tuple[float, int]] = []
        self._sim_time = 0.0

    def update(self, throttle: int, steering: int,
               max_speed: float, wheelbase: float, dt: float,
               turn_scale: float = 0.1, steer_deadband: float = 0.05):
        self._sim_time += dt

        # Steering lag
        if self._steer_lag > 0:
            self._steer_buf.append((self._sim_time, steering))
            target_time = self._sim_time - self._steer_lag
            delayed_steer = 1500
            for t, s in self._steer_buf:
                if t <= target_time:
                    delayed_steer = s
                else:
                    break
            while len(self._steer_buf) > 1 and self._steer_buf[0][0] < target_time - 0.1:
                self._steer_buf.pop(0)
            steering = delayed_steer

        # PPM → target wheel speeds (mm/s)
        forward_mms = (throttle - 1500) / 500.0 * max_speed * 1000.0

        steer_frac = (1500 - steering) / 500.0
        if abs(steer_frac) < steer_deadband:
            steer_frac = 0.0
        angle_deg = steer_frac * 25.0 / self._max_steer
        angle_deg = max(-25.0, min(25.0, angle_deg))
        v_speed_mm = angle_deg * (self._track_m * 1000.0) * math.pi / 180.0 * self._steer_scale

        left_target = forward_mms + v_speed_mm
        right_target = forward_mms - v_speed_mm

        # SmoothSpeed (acceleration limiter)
        ticks = dt * self._smooth_hz
        accel_cap = self._accel_cap * ticks

        left = self._smooth_one(self._last_left, left_target, accel_cap, ticks)
        right = self._smooth_one(self._last_right, right_target, accel_cap, ticks)

        # Angular velocity limit
        diff = abs(left - right)
        if diff > self._ang_diff_lim:
            scale = diff / self._ang_diff_lim
            left /= scale
            right /= scale

        # Clamp to max wheel speed (proportional)
        max_abs = max(abs(left), abs(right))
        if max_abs > self._max_wheel:
            scale = self._max_wheel / max_abs
            left *= scale
            right *= scale

        self._last_left = left
        self._last_right = right

        # Kinematics
        speed_mps = (left + right) / 2000.0
        omega = (left - right) / (self._track_m * 1000.0)

        lat_rad = math.radians(self.lat)
        cos_lat = math.cos(lat_rad) or 1e-9

        if self._rc_offset == 0.0:
            self.heading_rad += omega * dt
            self.lat += (speed_mps * math.cos(self.heading_rad) * dt) / 111320.0
            self.lon += (speed_mps * math.sin(self.heading_rad) * dt) / (111320.0 * cos_lat)
        else:
            rc_lat = self.lat + (self._rc_offset * math.cos(self.heading_rad)) / 111320.0
            rc_lon = self.lon + (self._rc_offset * math.sin(self.heading_rad)) / (111320.0 * cos_lat)
            self.heading_rad += omega * dt
            rc_lat += (speed_mps * math.cos(self.heading_rad) * dt) / 111320.0
            rc_lon += (speed_mps * math.sin(self.heading_rad) * dt) / (111320.0 * cos_lat)
            cos_lat2 = math.cos(math.radians(rc_lat)) or 1e-9
            self.lat = rc_lat - (self._rc_offset * math.cos(self.heading_rad)) / 111320.0
            self.lon = rc_lon - (self._rc_offset * math.sin(self.heading_rad)) / (111320.0 * cos_lat2)

        self.speed_mps = speed_mps

    def _smooth_one(self, last: float, target: float, accel_cap: float,
                    ticks: float) -> float:
        if last < target - accel_cap:
            return last + accel_cap
        elif last > target + accel_cap:
            decel_per_tick = abs(target - last) / self._decel_div
            result = last - decel_per_tick * ticks
            return max(result, target)  # never overshoot past target
        return target

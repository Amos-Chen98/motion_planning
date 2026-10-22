"""ROS-independent, speed-bounded references for entering the Dragon Dance."""

import math


def _finite(value, name):
    if isinstance(value, bool):
        raise ValueError("{} must be a finite number".format(name))
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError):
        raise ValueError("{} must be a finite number".format(name))
    if not math.isfinite(result):
        raise ValueError("{} must be a finite number".format(name))
    return result


def _vector3(values, name):
    try:
        result = tuple(_finite(value, name) for value in values)
    except TypeError:
        raise ValueError("{} must contain three finite numbers".format(name))
    if len(result) != 3:
        raise ValueError("{} must contain three finite numbers".format(name))
    return result


def yaw_rate_from_body_rates(roll, pitch, body_rates):
    """Return ZYX Euler yaw rate from body-frame angular velocity (p, q, r).

    Body angular velocity's z component alone is not the yaw derivative when
    the robot has roll or pitch. Euler yaw is undefined at pitch +/- pi/2.
    """
    roll = _finite(roll, "roll")
    pitch = _finite(pitch, "pitch")
    _, q, r = _vector3(body_rates, "body_rates")
    cosine_pitch = math.cos(pitch)
    if abs(cosine_pitch) < 1e-6:
        raise ValueError("yaw rate is undefined near a pitch singularity")
    return _finite(
        (math.sin(roll) * q + math.cos(roll) * r) / cosine_pitch,
        "yaw_rate",
    )


class AdaptiveEntryTrajectory:
    """Quintic position/yaw reference with bounded commanded speed.

    Endpoint accelerations and final velocities are zero; initial velocity is
    retained. For normalized time s, f' is bounded by 1.875 and |g'| by 1.
    Thus |delta| * 1.875 / T + |v0| bounds the entire reference's speed,
    including when v0 points away from or transverse to the displacement.

    Yaw is unwrapped along the shortest arc. The limits constrain the reference
    and do not imply that the physical robot tracks it within those limits.
    """

    def __init__(self, start_position, start_velocity, start_yaw, start_yaw_rate,
                 goal_position, goal_yaw, max_linear_speed, max_yaw_rate,
                 publish_rate):
        self.start_position = _vector3(start_position, "start_position")
        self.start_velocity = _vector3(start_velocity, "start_velocity")
        self.start_yaw = _finite(start_yaw, "start_yaw")
        self.start_yaw_rate = _finite(start_yaw_rate, "start_yaw_rate")
        self.goal_position = _vector3(goal_position, "goal_position")
        goal_yaw = _finite(goal_yaw, "goal_yaw")
        max_linear_speed = _finite(max_linear_speed, "max_linear_speed")
        max_yaw_rate = _finite(max_yaw_rate, "max_yaw_rate")
        publish_rate = _finite(publish_rate, "publish_rate")
        for name, value in (("max_linear_speed", max_linear_speed),
                            ("max_yaw_rate", max_yaw_rate),
                            ("publish_rate", publish_rate)):
            if value <= 0.0:
                raise ValueError("{} must be positive".format(name))

        linear_speed = math.hypot(*self.start_velocity)
        if linear_speed >= max_linear_speed:
            raise ValueError("start_velocity must be below max_linear_speed")
        if abs(self.start_yaw_rate) >= max_yaw_rate:
            raise ValueError("start_yaw_rate must be below max_yaw_rate")

        self._displacement = tuple(
            _finite(goal - start, "position displacement")
            for start, goal in zip(self.start_position, self.goal_position)
        )
        self.distance = _finite(math.hypot(*self._displacement), "distance")
        yaw_difference = _finite(goal_yaw - self.start_yaw, "yaw difference")
        self.yaw_delta = math.atan2(math.sin(yaw_difference), math.cos(yaw_difference))
        self.goal_yaw = self.start_yaw + self.yaw_delta
        durations = (
            ("linear_speed", 1.875 * self.distance / (max_linear_speed - linear_speed)),
            ("yaw_rate", 1.875 * abs(self.yaw_delta) / (max_yaw_rate - abs(self.start_yaw_rate))),
            ("publish_rate", 2.0 / publish_rate),
        )
        self.limiting_factor, self.duration = max(durations, key=lambda item: item[1])
        _finite(self.duration, "entry duration")
        if self.duration <= 0.0:
            raise ValueError("entry duration must be positive")

    def sample(self, elapsed):
        """Return (position, velocity, yaw, yaw_rate), clamped to the endpoints."""
        elapsed = _finite(elapsed, "elapsed")
        if elapsed <= 0.0:
            return (self.start_position, self.start_velocity,
                    self.start_yaw, self.start_yaw_rate)
        if elapsed >= self.duration:
            return self.goal_position, (0.0, 0.0, 0.0), self.goal_yaw, 0.0

        s = elapsed / self.duration
        f = s ** 3 * (10.0 + s * (-15.0 + 6.0 * s))
        g = s * (1.0 + s ** 2 * (-6.0 + s * (8.0 - 3.0 * s)))
        f_dot = 30.0 * s ** 2 * (1.0 - s) ** 2 / self.duration
        g_dot = 1.0 + s ** 2 * (-18.0 + s * (32.0 - 15.0 * s))
        position = tuple(
            start + delta * f + self.duration * velocity * g
            for start, delta, velocity in zip(
                self.start_position, self._displacement, self.start_velocity)
        )
        velocity = tuple(
            delta * f_dot + initial_velocity * g_dot
            for delta, initial_velocity in zip(self._displacement, self.start_velocity)
        )
        yaw = self.start_yaw + self.yaw_delta * f + self.duration * self.start_yaw_rate * g
        yaw_rate = self.yaw_delta * f_dot + self.start_yaw_rate * g_dot
        return position, velocity, yaw, yaw_rate

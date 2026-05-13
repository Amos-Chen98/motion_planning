#!/usr/bin/env python3

import math

import rospy
import tf


EDF_FRAMES = [
    "dragon/edf1_left",
    "dragon/edf1_right",
    "dragon/edf2_left",
    "dragon/edf2_right",
    "dragon/edf3_left",
    "dragon/edf3_right",
    "dragon/edf4_left",
    "dragon/edf4_right",
]


def normalize_frame(frame_id):
    return frame_id[1:] if frame_id.startswith("/") else frame_id


class DragonEdfPositionPrinter:
    NODE_NAME = "print_dragon_edf_positions"
    DEFAULT_PRINT_RATE_HZ = 5.0
    DEFAULT_REFERENCE_FRAME = "world"
    DEFAULT_OVERLAP_REFERENCE_FRAME = "dragon/cog"
    DEFAULT_LOOKUP_TIMEOUT_SEC = 0.05
    DEFAULT_EDF_RADIUS = 0.035
    DEFAULT_EDF_MAX_TILT = 0.26
    LOG_THROTTLE_SEC = 2.0

    def __init__(self):
        rospy.init_node(self.NODE_NAME, anonymous=False)

        self.print_rate_hz = float(rospy.get_param("~print_rate_hz", self.DEFAULT_PRINT_RATE_HZ))
        self.reference_frame = normalize_frame(
            str(rospy.get_param("~reference_frame", self.DEFAULT_REFERENCE_FRAME))
        )
        self.overlap_reference_frame = normalize_frame(
            str(rospy.get_param("~overlap_reference_frame", self.DEFAULT_OVERLAP_REFERENCE_FRAME))
        )
        lookup_timeout_sec = float(rospy.get_param("~lookup_timeout_sec", self.DEFAULT_LOOKUP_TIMEOUT_SEC))
        self.edf_radius = self.get_param_with_global_fallback(
            "~edf_radius",
            "/dragon/edf_radius",
            self.DEFAULT_EDF_RADIUS,
        )
        self.edf_max_tilt = self.get_param_with_global_fallback(
            "~edf_max_tilt",
            "/dragon/edf_max_tilt",
            self.DEFAULT_EDF_MAX_TILT,
        )

        if self.print_rate_hz <= 0.0:
            raise ValueError("~print_rate_hz must be positive.")
        if lookup_timeout_sec < 0.0:
            raise ValueError("~lookup_timeout_sec must be non-negative.")
        if not math.isfinite(self.edf_radius) or self.edf_radius < 0.0:
            raise ValueError("~edf_radius must be a finite non-negative value.")
        if not math.isfinite(self.edf_max_tilt):
            raise ValueError("~edf_max_tilt must be finite.")

        self.lookup_timeout = rospy.Duration.from_sec(lookup_timeout_sec)
        self.tf_listener = tf.TransformListener()

        rospy.loginfo(
            "Printing dragon EDF positions in frame '%s' and overlap clearance in frame '%s' at %.3f Hz "
            "(edf_radius=%.4f m, edf_max_tilt=%.4f rad)",
            self.reference_frame,
            self.overlap_reference_frame,
            self.print_rate_hz,
            self.edf_radius,
            self.edf_max_tilt,
        )

    @staticmethod
    def get_param_with_global_fallback(private_name, global_name, default_value):
        if rospy.has_param(private_name):
            return float(rospy.get_param(private_name))
        return float(rospy.get_param(global_name, default_value))

    def lookup_position(self, target_frame, edf_frame):
        try:
            self.tf_listener.waitForTransform(
                target_frame,
                edf_frame,
                rospy.Time(0),
                self.lookup_timeout,
            )
            translation, _ = self.tf_listener.lookupTransform(
                target_frame,
                edf_frame,
                rospy.Time(0),
            )
            return translation, None
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as exc:
            return None, exc

    def collect_positions(self, target_frame):
        positions = {}
        unavailable = []

        for edf_frame in EDF_FRAMES:
            position, error = self.lookup_position(target_frame, edf_frame)
            positions[edf_frame] = position
            if error is not None:
                unavailable.append("{} ({})".format(edf_frame, error))

        if unavailable:
            rospy.logwarn_throttle(
                self.LOG_THROTTLE_SEC,
                "Unavailable EDF TF in frame '%s': %s",
                target_frame,
                "; ".join(unavailable),
            )

        return positions

    def find_min_overlap_clearance(self, positions):
        min_overlap = None

        for i, edf_i in enumerate(EDF_FRAMES):
            position_i = positions[edf_i]
            if position_i is None:
                continue

            for j in range(i + 1, len(EDF_FRAMES)):
                if i // 2 == j // 2:
                    continue

                edf_j = EDF_FRAMES[j]
                position_j = positions[edf_j]
                if position_j is None:
                    continue

                dx = position_i[0] - position_j[0]
                dy = position_i[1] - position_j[1]
                dz = position_i[2] - position_j[2]
                projected_dist = math.sqrt(dx * dx + dy * dy)
                threshold = 2.0 * self.edf_radius + abs(dz) * math.tan(self.edf_max_tilt)
                clearance = projected_dist - threshold

                if min_overlap is None or clearance < min_overlap["clearance"]:
                    min_overlap = {
                        "clearance": clearance,
                        "projected_dist": projected_dist,
                        "threshold": threshold,
                        "dz": dz,
                        "edf_i": edf_i,
                        "edf_j": edf_j,
                    }

        return min_overlap

    def format_positions(self, positions):
        lines = ["Dragon EDF positions in '{}':".format(self.reference_frame)]
        for edf_frame in EDF_FRAMES:
            position = positions[edf_frame]
            if position is None:
                lines.append("  {}: unavailable".format(edf_frame))
            else:
                lines.append(
                    "  {}: x={:.4f} m, y={:.4f} m, z={:.4f} m".format(
                        edf_frame,
                        position[0],
                        position[1],
                        position[2],
                    )
                )
        return "\n".join(lines)

    def format_overlap_clearance(self, positions):
        unavailable = [edf_frame for edf_frame in EDF_FRAMES if positions[edf_frame] is None]
        if unavailable:
            return "Propeller overlap clearance in '{}': unavailable (missing TF: {})".format(
                self.overlap_reference_frame,
                ", ".join(unavailable),
            )

        min_overlap = self.find_min_overlap_clearance(positions)
        if min_overlap is None:
            return "Propeller overlap clearance in '{}': unavailable".format(self.overlap_reference_frame)

        status = "OVERLAPPED" if min_overlap["clearance"] < 0.0 else "clear"
        return (
            "Propeller overlap clearance in '{}': min={:.4f} m, status={}, pair={} <-> {}, "
            "projected_dist={:.4f} m, threshold={:.4f} m, dz={:.4f} m"
        ).format(
            self.overlap_reference_frame,
            min_overlap["clearance"],
            status,
            min_overlap["edf_i"],
            min_overlap["edf_j"],
            min_overlap["projected_dist"],
            min_overlap["threshold"],
            min_overlap["dz"],
        )

    def run(self):
        rate = rospy.Rate(self.print_rate_hz)
        while not rospy.is_shutdown():
            positions = self.collect_positions(self.reference_frame)
            if self.overlap_reference_frame == self.reference_frame:
                overlap_positions = positions
            else:
                overlap_positions = self.collect_positions(self.overlap_reference_frame)

            rospy.loginfo(
                "%s\n%s",
                self.format_positions(positions),
                self.format_overlap_clearance(overlap_positions),
            )
            rate.sleep()


def main():
    try:
        DragonEdfPositionPrinter().run()
    except rospy.ROSInterruptException:
        pass
    except ValueError as exc:
        rospy.logerr("Invalid dragon EDF position printer configuration: %s", exc)


if __name__ == "__main__":
    main()

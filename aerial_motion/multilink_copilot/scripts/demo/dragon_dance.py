#!/usr/bin/env python3

import math
import select
import sys
import termios
import tty

import rospy
from geometry_msgs.msg import PoseStamped


class DragonDancePublisher:
    TOPIC_NAME = "root/target_pose"
    FRAME_ID = "world"
    PUBLISH_RATE_HZ = 40.0
    TOTAL_DURATION = 60.0

    ELLIPSE_CENTER_X = 0.0
    ELLIPSE_CENTER_Y = -0.7
    ELLIPSE_SEMI_MAJOR = 2.0
    ELLIPSE_SEMI_MINOR = 1.25

    Z_OFFSET = 1.3
    Z_AMPLITUDE = 0.5
    Z_CYCLES = 3.0

    def __init__(self):
        rospy.init_node("dragon_dance", anonymous=False)

        self.publisher = rospy.Publisher(self.TOPIC_NAME, PoseStamped, queue_size=1)
        self.samples_per_cycle = int(round(self.TOTAL_DURATION * self.PUBLISH_RATE_HZ))
        self.sample_index = 0
        self.completed_cycles = 0
        self.stdin_fd = None
        self.stdin_settings = None
        self.keyboard_enabled = self.configure_keyboard_input()

        rospy.on_shutdown(self.restore_terminal_settings)

        rospy.loginfo(
            "Publishing dragon dance trajectory on %s at %.1f Hz with %d samples per cycle",
            self.publisher.resolved_name,
            self.PUBLISH_RATE_HZ,
            self.samples_per_cycle,
        )

    def run(self):
        rate = rospy.Rate(self.PUBLISH_RATE_HZ)
        try:
            while not rospy.is_shutdown():
                if self.key_pressed():
                    rospy.loginfo("Key press detected, stopping trajectory publisher")
                    break

                elapsed = self.sample_index / self.PUBLISH_RATE_HZ
                self.publisher.publish(self.build_pose_message(elapsed))

                self.sample_index += 1
                if self.sample_index >= self.samples_per_cycle:
                    self.sample_index = 0
                    self.completed_cycles += 1
                    rospy.loginfo("Completed trajectory cycle %d, restarting", self.completed_cycles)

                rate.sleep()
        finally:
            self.restore_terminal_settings()

    def build_pose_message(self, elapsed):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = self.FRAME_ID

        angular_speed = 2.0 * math.pi / self.TOTAL_DURATION
        z_angular_speed = 2.0 * math.pi * self.Z_CYCLES / self.TOTAL_DURATION
        theta = (math.pi / 2.0) - angular_speed * elapsed

        pose_msg.pose.position.x = self.ELLIPSE_CENTER_X + self.ELLIPSE_SEMI_MAJOR * math.cos(theta)
        pose_msg.pose.position.y = self.ELLIPSE_CENTER_Y + self.ELLIPSE_SEMI_MINOR * math.sin(theta)

        z_phase = z_angular_speed * elapsed - (math.pi / 2.0)
        pose_msg.pose.position.z = self.Z_OFFSET + self.Z_AMPLITUDE * math.sin(z_phase)

        velocity_x = self.ELLIPSE_SEMI_MAJOR * angular_speed * math.sin(theta)
        velocity_y = -self.ELLIPSE_SEMI_MINOR * angular_speed * math.cos(theta)
        velocity_z = self.Z_AMPLITUDE * z_angular_speed * math.cos(z_phase)

        horizontal_speed = math.hypot(velocity_x, velocity_y)
        yaw = math.atan2(velocity_y, velocity_x)
        # The target pose is applied at the first-link tail, so pitch must be flipped.
        pitch = math.atan2(velocity_z, horizontal_speed)

        half_pitch = 0.5 * pitch
        half_yaw = 0.5 * yaw
        pose_msg.pose.orientation.x = -math.sin(half_pitch) * math.sin(half_yaw)
        pose_msg.pose.orientation.y = math.sin(half_pitch) * math.cos(half_yaw)
        pose_msg.pose.orientation.z = math.cos(half_pitch) * math.sin(half_yaw)
        pose_msg.pose.orientation.w = math.cos(half_pitch) * math.cos(half_yaw)

        return pose_msg

    def configure_keyboard_input(self):
        if not sys.stdin.isatty():
            rospy.logwarn("No interactive terminal detected, repeating until the node is stopped externally")
            return False

        try:
            self.stdin_fd = sys.stdin.fileno()
            self.stdin_settings = termios.tcgetattr(self.stdin_fd)
            tty.setcbreak(self.stdin_fd)
        except (termios.error, ValueError, OSError) as exc:
            rospy.logwarn("Failed to enable keyboard stop input: %s", exc)
            self.stdin_fd = None
            self.stdin_settings = None
            return False

        rospy.loginfo("Press any key in this terminal to stop the repeating trajectory publisher")
        return True

    def restore_terminal_settings(self):
        if self.stdin_fd is None or self.stdin_settings is None:
            return

        termios.tcsetattr(self.stdin_fd, termios.TCSADRAIN, self.stdin_settings)
        self.stdin_fd = None
        self.stdin_settings = None

    def key_pressed(self):
        if not self.keyboard_enabled:
            return False

        readable, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not readable:
            return False

        sys.stdin.read(1)
        return True


if __name__ == "__main__":
    try:
        DragonDancePublisher().run()
    except rospy.ROSInterruptException:
        pass

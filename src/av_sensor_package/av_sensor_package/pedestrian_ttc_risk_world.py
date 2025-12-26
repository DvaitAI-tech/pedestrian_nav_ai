#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray

import matplotlib.pyplot as plt
from collections import defaultdict, deque
import math
import time


class PedestrianWorldCollisionTTC(Node):
    def __init__(self):
        super().__init__('pedestrian_world_collision_ttc')

        # ---------- ROS ----------
        self.sub = self.create_subscription(
            PoseArray,
            '/perception/pedestrians_world',
            self.callback,
            10
        )

        # ---------- Parameters ----------
        self.history_len = 20
        self.alpha = 0.3                # velocity smoothing
        self.collision_radius = 1.5     # ego safety radius (m)
        self.ttc_warn = 5.0
        self.ttc_crit = 2.5
        self.track_timeout = 1.0        # seconds to keep track if missing

        # ---------- Tracking ----------
        self.tracks = defaultdict(lambda: deque(maxlen=self.history_len))
        self.vel_smooth = defaultdict(lambda: (0.0, 0.0))
        self.last_seen = {}
        self.next_id = 0

        # ---------- Plot ----------
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(9, 9))
        self.last_draw = time.time()

        self.get_logger().info("✅ Pedestrian World Collision TTC Node Started")

    # =========================================================
    # CALLBACK
    # =========================================================
    def callback(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9

        detections = [(pose.position.x, pose.position.y) for pose in msg.poses]

        # ---------- Associate detections to tracks (nearest neighbor) ----------
        assigned = set()
        for x, y in detections:
            best_id = None
            best_dist = 2.0  # meters

            for pid, hist in self.tracks.items():
                px, py, _ = hist[-1]
                d = math.hypot(x - px, y - py)
                if d < best_dist:
                    best_dist = d
                    best_id = pid

            if best_id is not None:
                # Update existing track
                px, py, pt = self.tracks[best_id][-1]
                dt = now - pt

                if dt > 0.001:
                    raw_vx = (x - px) / dt
                    raw_vy = (y - py) / dt
                    vx_p, vy_p = self.vel_smooth[best_id]

                    self.vel_smooth[best_id] = (
                        self.alpha * raw_vx + (1 - self.alpha) * vx_p,
                        self.alpha * raw_vy + (1 - self.alpha) * vy_p
                    )

                self.tracks[best_id].append((x, y, now))
                self.last_seen[best_id] = now
                assigned.add(best_id)

            else:
                # New track
                pid = self.next_id
                self.next_id += 1

                self.tracks[pid].append((x, y, now))
                self.vel_smooth[pid] = (0.0, 0.0)
                self.last_seen[pid] = now

        # ---------- Remove stale tracks ----------
        to_delete = []
        for pid, last_t in self.last_seen.items():
            if now - last_t > self.track_timeout:
                to_delete.append(pid)

        for pid in to_delete:
            del self.tracks[pid]
            del self.vel_smooth[pid]
            del self.last_seen[pid]

        # ---------- Draw ----------
        if time.time() - self.last_draw > 0.1:
            self.draw()
            self.last_draw = time.time()

    # =========================================================
    # TTC COMPUTATION
    # =========================================================
    def compute_ttc(self, x, y, vx, vy):
        """
        True collision-based TTC.
        Ego at (0,0).
        """
        dist = math.hypot(x, y)

        if dist <= self.collision_radius:
            return 0.0

        closing_speed = -(x * vx + y * vy) / (dist + 1e-6)

        if closing_speed <= 0:
            return math.inf

        ttc = (dist - self.collision_radius) / closing_speed
        return ttc if ttc > 0 else math.inf

    # =========================================================
    # DRAW
    # =========================================================
    def draw(self):
        self.ax.clear()

        # ---------- Axes ----------
        self.ax.set_title("Pedestrian Collision Risk (World / Fused Data)")
        self.ax.set_xlabel("Lateral Y (m)")
        self.ax.set_ylabel("Longitudinal X (m)")
        self.ax.set_xlim(-100, 100)
        self.ax.set_ylim(-100, 100)
        self.ax.set_aspect('equal')
        self.ax.grid(True)

        # ---------- Ego ----------
        self.ax.plot(0, 0, 'k^', markersize=15)
        ego_zone = plt.Circle((0, 0), self.collision_radius,
                              color='red', alpha=0.15)
        self.ax.add_patch(ego_zone)

        # ---------- Pedestrians ----------
        for pid, hist in self.tracks.items():
            if len(hist) < 2:
                continue

            x, y, _ = hist[-1]
            vx, vy = self.vel_smooth[pid]
            ttc = self.compute_ttc(x, y, vx, vy)

            if ttc < self.ttc_crit:
                color, risk = 'red', 'HIGH'
            elif ttc < self.ttc_warn:
                color, risk = 'orange', 'MED'
            else:
                color, risk = 'green', 'LOW'

            # Past trajectory
            xs = [p[0] for p in hist]
            ys = [p[1] for p in hist]
            self.ax.plot(ys, xs, '--', color=color, linewidth=1)

            # Current position
            self.ax.plot(y, x, 'o', color=color, markersize=9)

            # Velocity arrow
            self.ax.arrow(
                y, x,
                vy * 2.0, vx * 2.0,
                head_width=1.2,
                head_length=1.5,
                fc=color, ec=color
            )

            # Label
            label = (
                f"ID {pid}\n"
                f"x={x:.1f} y={y:.1f} m\n"
                f"v=({vx:.1f},{vy:.1f}) m/s\n"
                f"TTC={ttc:.1f}s\n{risk}"
            )

            self.ax.text(
                y + 1.5, x + 1.5,
                label,
                fontsize=8,
                bbox=dict(facecolor='white', alpha=0.75, edgecolor=color)
            )

        # ---------- Explanation Box ----------
        explain = (
            "Imagine standing on the car roof, looking down:\n"
            "▲  Ego vehicle (you)\n"
            "●  Pedestrian (current position)\n"
            "---  Past trajectory (where they came from)\n"
            "→  Velocity vector (where they are moving)\n"
            "Color = Risk (Green=Safe, Orange=Warning, Red=Danger)"
        )

        self.ax.text(
            0.5, -0.20, explain,
            transform=self.ax.transAxes,
            ha='center', va='top',
            fontsize=9,
            bbox=dict(facecolor='white', alpha=0.95, edgecolor='black')
        )

        plt.pause(0.001)


# =========================================================
# MAIN
# =========================================================
def main(args=None):
    rclpy.init(args=args)
    node = PedestrianWorldCollisionTTC()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

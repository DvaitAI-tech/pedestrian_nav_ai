#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray

import matplotlib.pyplot as plt
from collections import defaultdict, deque
import math
import time


class PedestrianVelocityPlotter(Node):
    def __init__(self):
        super().__init__('pedestrian_velocity_plotter')

        self.sub = self.create_subscription(
            Detection2DArray,
            '/perception/tracked_pedestrians',
            self.callback,
            10
        )

        # Store history per ID: (x, y, t)
        self.history_len = 30
        self.tracks = defaultdict(lambda: deque(maxlen=self.history_len))

        # --- Matplotlib ---
        plt.ion()
        self.fig, (self.ax_traj, self.ax_speed) = plt.subplots(1, 2, figsize=(12, 5))

        self.ax_traj.set_title("Pedestrian Trajectories + Velocity")
        self.ax_traj.set_xlabel("X (pixels)")
        self.ax_traj.set_ylabel("Y (pixels)")
        self.ax_traj.invert_yaxis()

        self.ax_speed.set_title("Pedestrian Speed vs Time")
        self.ax_speed.set_xlabel("Time (s)")
        self.ax_speed.set_ylabel("Speed (pixels/s)")

        self.last_draw = time.time()

        self.get_logger().info("Pedestrian velocity plotter started.")

    def callback(self, msg):
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        for det in msg.detections:
            if not det.id:
                continue

            pid = det.id
            x = det.bbox.center.position.x
            y = det.bbox.center.position.y

            self.tracks[pid].append((x, y, now))

        # limit redraw rate
        if time.time() - self.last_draw > 0.1:
            self.draw()
            self.last_draw = time.time()

    def draw(self):
        self.ax_traj.clear()
        self.ax_speed.clear()

        self.ax_traj.set_title("Pedestrian Trajectories + Velocity")
        self.ax_traj.set_xlabel("X (pixels)")
        self.ax_traj.set_ylabel("Y (pixels)")
        self.ax_traj.invert_yaxis()

        self.ax_speed.set_title("Pedestrian Speed vs Time")
        self.ax_speed.set_xlabel("Time (s)")
        self.ax_speed.set_ylabel("Speed (pixels/s)")

        for pid, data in self.tracks.items():
            if len(data) < 2:
                continue

            xs, ys, ts = zip(*data)

            # --- Trajectory ---
            self.ax_traj.plot(xs, ys, marker='o', label=f"ID {pid}")

            # --- Velocity (last segment only, arrow) ---
            x1, y1, t1 = data[-2]
            x2, y2, t2 = data[-1]
            dt = t2 - t1
            if dt > 0:
                vx = (x2 - x1) / dt
                vy = (y2 - y1) / dt

                self.ax_traj.arrow(
                    x1, y1,
                    vx * 0.1, vy * 0.1,
                    head_width=3,
                    length_includes_head=True
                )

            # --- Speed vs time ---
            speeds = []
            times = []
            for i in range(1, len(data)):
                dx = data[i][0] - data[i - 1][0]
                dy = data[i][1] - data[i - 1][1]
                dt = data[i][2] - data[i - 1][2]
                if dt > 0:
                    speed = math.sqrt(dx * dx + dy * dy) / dt
                    speeds.append(speed)
                    times.append(data[i][2] - data[0][2])

            if speeds:
                self.ax_speed.plot(times, speeds, label=f"ID {pid}")

        self.ax_traj.legend(fontsize='small')
        self.ax_speed.legend(fontsize='small')

        plt.pause(0.001)


def main(args=None):
    rclpy.init(args=args)
    node = PedestrianVelocityPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

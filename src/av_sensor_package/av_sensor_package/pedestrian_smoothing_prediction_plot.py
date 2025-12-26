#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray

import matplotlib.pyplot as plt
from collections import defaultdict, deque
import math
import time


class PedestrianPredictionPlotter(Node):
    def __init__(self):
        super().__init__('pedestrian_prediction_plotter')

        self.sub = self.create_subscription(
            Detection2DArray,
            '/perception/tracked_pedestrians',
            self.callback,
            10
        )

        # Parameters
        self.history_len = 30
        self.alpha = 0.3          # EMA smoothing factor
        self.pred_horizon = 2.0   # seconds
        self.pred_steps = 10

        # Track storage: (x, y, t)
        self.tracks = defaultdict(lambda: deque(maxlen=self.history_len))

        # Smoothed velocity per ID
        self.v_smooth = defaultdict(lambda: (0.0, 0.0))

        # Matplotlib
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(7, 6))
        self.ax.set_title("Pedestrian Trajectory + Smoothed Prediction")
        self.ax.set_xlabel("X (pixels)")
        self.ax.set_ylabel("Y (pixels)")
        self.ax.invert_yaxis()

        self.last_draw = time.time()
        self.get_logger().info("Smoothing + prediction plotter started.")

    def callback(self, msg):
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        for det in msg.detections:
            if not det.id:
                continue

            pid = det.id
            x = det.bbox.center.position.x
            y = det.bbox.center.position.y

            self.tracks[pid].append((x, y, now))

            if len(self.tracks[pid]) >= 2:
                x1, y1, t1 = self.tracks[pid][-2]
                x2, y2, t2 = self.tracks[pid][-1]
                dt = t2 - t1
                if dt > 0:
                    vx = (x2 - x1) / dt
                    vy = (y2 - y1) / dt

                    # EMA smoothing
                    vx_prev, vy_prev = self.v_smooth[pid]
                    vx_s = self.alpha * vx + (1 - self.alpha) * vx_prev
                    vy_s = self.alpha * vy + (1 - self.alpha) * vy_prev
                    self.v_smooth[pid] = (vx_s, vy_s)

        if time.time() - self.last_draw > 0.1:
            self.draw()
            self.last_draw = time.time()

    def draw(self):
        self.ax.clear()
        self.ax.set_title("Pedestrian Trajectory + Smoothed Prediction")
        self.ax.set_xlabel("X (pixels)")
        self.ax.set_ylabel("Y (pixels)")
        self.ax.invert_yaxis()

        for pid, data in self.tracks.items():
            if len(data) < 3:
                continue

            xs, ys, ts = zip(*data)
            self.ax.plot(
                        xs, ys,
                        linestyle='-',
                        linewidth=2,
                        marker='o',
                        markersize=4,
                        alpha=0.9,
                        label=f"ID {pid} (past)"
                    )


            # Smoothed velocity arrow
            vx_s, vy_s = self.v_smooth[pid]
            x, y, t = data[-1]

            self.ax.arrow(
                x, y,
                vx_s * 0.2, vy_s * 0.2,
                head_width=3,
                color='black',
                length_includes_head=True
            )

            # Future prediction (CV)
            future_x = []
            future_y = []

            for i in range(1, self.pred_steps + 1):
                dt = (i / self.pred_steps) * self.pred_horizon
                future_x.append(x + vx_s * dt)
                future_y.append(y + vy_s * dt)

            self.ax.plot(
                future_x, future_y,
                linestyle='--',
                linewidth=2
            )

        self.ax.legend(fontsize='small')
        # ---- In-graph explanation ----
        legend_text = (
            "Solid line + dots : Observed past trajectory\n"
            "Red arrow        : Smoothed velocity direction\n"
            "Black dashed     : Predicted future path (CV model)"
        )
        self.ax.text(
            0.02, 0.02,
            legend_text,
            transform=self.ax.transAxes,
            fontsize=9,
            verticalalignment='bottom',
            bbox=dict(boxstyle="round", facecolor="white", alpha=0.8)
        )
        plt.pause(0.001)
        


def main(args=None):
    rclpy.init(args=args)
    node = PedestrianPredictionPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

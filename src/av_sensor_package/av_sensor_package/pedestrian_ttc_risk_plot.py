#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray

import matplotlib.pyplot as plt
from collections import defaultdict, deque
import math
import time


class PedestrianTTCRisk(Node):
    def __init__(self):
        super().__init__('pedestrian_ttc_risk')

        self.sub = self.create_subscription(
            Detection2DArray,
            '/perception/tracked_pedestrians',
            self.callback,
            10
        )
        self.ego_x = None
        self.ego_y = None
        # Parameters
        self.history_len = 30
        self.alpha = 0.3
        self.pred_horizon = 3.0
        self.pred_steps = 15
        self.safety_radius = 100  # pixels

        self.tracks = defaultdict(lambda: deque(maxlen=self.history_len))
        self.v_smooth = defaultdict(lambda: (0.0, 0.0))

        # Ego position (image-space bottom center)
        # self.ego_x = 640 / 2
        # self.ego_y = 480

        # Plot
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(7, 6))
        self.ax.invert_yaxis()

        self.last_draw = time.time()
        self.get_logger().info("TTC & Risk estimator started.")

    def callback(self, msg):
        
        # Initialize ego position ONLY when detections are available
        if self.ego_x is None and len(msg.detections) > 0:
            xs = [det.bbox.center.position.x for det in msg.detections]
            ys = [det.bbox.center.position.y for det in msg.detections]

            self.ego_x = sum(xs) / len(xs)          # image center approximation
            self.ego_y = max(ys) + 120               # below pedestrians

            self.get_logger().info(
                f"Ego reference initialized at (x={self.ego_x:.1f}, y={self.ego_y:.1f})"
            )

        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        for det in msg.detections:
            pid = det.id
            if not pid:
                continue

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
                    vx_p, vy_p = self.v_smooth[pid]
                    #momentum aver
                    self.v_smooth[pid] = (
                        self.alpha * vx + (1 - self.alpha) * vx_p,
                        self.alpha * vy + (1 - self.alpha) * vy_p
                    )
        
        if self.ego_x is None:
            return
        if time.time() - self.last_draw > 0.1:

            self.draw()
            self.last_draw = time.time()

    def compute_ttc(self, x, y, vx, vy):
        """
        Continuous-time TTC based on longitudinal (image Y) distance.
        Image Y increases downward.
        Ego is below pedestrians.
        """

        # Ego not initialized
        if self.ego_x is None or self.ego_y is None:
            return math.inf

        # Current longitudinal distance to ego
        dy_now = self.ego_y - y

        # Case 1: already unsafe
        if dy_now <= self.safety_radius:
            return 0.0

        # Case 2: pedestrian not approaching ego
        # vy <= 0 means moving away or sideways
        # if vy <= 0:
        #     return math.inf

        # Case 3: approaching → time to enter safety zone
        ttc = (dy_now - self.safety_radius) / vy

        # Numerical safety
        if ttc < 0:
            return math.inf

        return ttc



    def draw(self):
        self.ax.clear()
        self.ax.set_title("Pedestrian TTC & Risk Estimation")
        self.ax.set_xlabel("X (pixels)")
        self.ax.set_ylabel("Y (pixels)")
        self.ax.invert_yaxis()

        # Ego marker
        self.ax.plot(self.ego_x, self.ego_y, 'ks', markersize=8, label="Ego")

        for pid, data in self.tracks.items():
            if len(data) < 3:
                continue

            xs, ys, ts = zip(*data)
            x, y, _ = data[-1]
            vx, vy = self.v_smooth[pid]

            ttc = self.compute_ttc(x, y, vx, vy)

            # Risk level
            if ttc < 2.0:
                color = 'red'
                risk = 'HIGH'
            elif ttc < 5.0:
                color = 'orange'
                risk = 'MED'
            else:
                color = 'green'
                risk = 'LOW'

            # Past trajectory
            self.ax.plot(xs, ys, color=color, linewidth=2)

            # Velocity arrow
            self.ax.arrow(
                x, y,
                vx * 0.2, vy * 0.2,
                color='black',
                head_width=6,
                length_includes_head=True
            )

            # Label
            label = f"ID {pid}\nTTC: {ttc:.1f}s\n{risk}"
            self.ax.text(x, y - 10, label, fontsize=8, color=color)

        # Explanation box
        self.ax.text(
            0.02, 0.02,
            "Solid line: Past trajectory\n"
            "Arrow: Smoothed velocity\n"
            "Color: Risk (Green/Orange/Red)\n"
            "TTC = Time to collision",
            transform=self.ax.transAxes,
            fontsize=9,
            bbox=dict(facecolor='white', alpha=0.8)
        )
        # print( f"ID {pid} | dy={self.ego_y - y:.1f} | vy={vy:.1f} | TTC={ttc:.2f}")

        self.ax.legend()
        plt.pause(0.001)


def main(args=None):
    rclpy.init(args=args)
    node = PedestrianTTCRisk()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

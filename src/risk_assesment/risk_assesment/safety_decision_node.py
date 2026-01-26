#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import time
from collections import defaultdict, deque

from geometry_msgs.msg import PoseArray, Twist
from std_msgs.msg import String

class SafetyDecisionNode(Node):
    def __init__(self):
        super().__init__('safety_decision_node')

        # === PARAMETERS ===
        self.stop_dist = 30.0       
        self.slow_ttc = 35.0        
        self.stop_ttc = 10.0         
        self.lane_width = 10.0      # meters (total width)
        self.collision_radius = 20.5 

        # === TRACKING STATE (Synced with Visualizer logic) ===
        self.tracks = defaultdict(lambda: deque(maxlen=5)) # (x, y, timestamp)
        self.vel_smooth = defaultdict(lambda: (0.0, 0.0))
        self.last_seen = {}
        self.next_id = 0
        self.alpha = 0.3            # Velocity smoothing factor
        self.track_timeout = 1.0    # Seconds to keep a track
        self.ego_speed = 0.0        
        # Subscribers
        self.sub_peds = self.create_subscription(PoseArray, '/perception/pedestrians_world', self.ped_callback, 10)
        self.sub_ego = self.create_subscription(Twist, '/ego/cmd_vel', self.ego_callback, 10)

        # Publisher
        self.pub = self.create_publisher(String, '/av/decision_state', 10)

        self.get_logger().info("🚦 Advanced Safety Decision Node with Tracking Started")

    def ego_callback(self, msg: Twist):
        self.ego_speed = abs(msg.linear.y)

    def ped_callback(self, msg: PoseArray):
        now = self.get_clock().now().nanoseconds / 1e9
        detections = [(pose.position.x, pose.position.y) for pose in msg.poses]
        
        # 1. TRACKING & VELOCITY ESTIMATION (Nearest Neighbor)
        current_active_ids = []
        for x, y in detections:
            best_id = None
            best_dist = 3.0 # Association gate in meters

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
            else:
                # New track
                best_id = self.next_id
                self.next_id += 1
                self.tracks[best_id].append((x, y, now))
                self.vel_smooth[best_id] = (0.0, 0.0)
                self.last_seen[best_id] = now
            
            current_active_ids.append(best_id)

        # 2. DECISION LOGIC
        decision = "GO"
        critical_id = -1
        min_ttc = float('inf')

        for pid in current_active_ids:
            x, y, _ = self.tracks[pid][-1]
            vx, vy = self.vel_smooth[pid]

            print(f"Evaluating Pedestrian ID: {pid} at Position ({x:.2f}, {y:.2f}) with Velocity ({vx:.2f}, {vy:.2f}) {self.lane_width / 2.0}")
            # Lane Filter
            if y <= 0.0 or abs(x) > (self.lane_width / 2.0):
                continue
            # Advanced TTC Calculation
            dist = math.hypot(x, y)
            # Closing speed includes ego speed + pedestrian relative velocity
            closing_speed = self.ego_speed - vy
            
            ttc = float('inf')
            if closing_speed > 0:
                ttc = (dist - self.collision_radius) / closing_speed

            # Selection of most critical object
            if ttc < min_ttc:
                min_ttc = ttc
                critical_id = pid

        # Final Decision Thresholds
        if min_ttc < self.stop_ttc or (critical_id != -1 and math.hypot(self.tracks[critical_id][-1][0], self.tracks[critical_id][-1][1]) < self.stop_dist):
            decision = "STOP"
        elif min_ttc < self.slow_ttc:
            decision = "SLOW"

        self.publish_decision(decision, critical_id, min_ttc)

        # Cleanup stale tracks
        self.cleanup_tracks(now)

    def cleanup_tracks(self, now):
        to_delete = [pid for pid, t in self.last_seen.items() if now - t > self.track_timeout]
        for pid in to_delete:
            del self.tracks[pid]
            del self.vel_smooth[pid]
            del self.last_seen[pid]

    def publish_decision(self, decision, pid, ttc):
        msg = String()
        msg.data = f"{decision}|ID:{pid}"
        self.pub.publish(msg)
        
        if pid != -1:
            self.get_logger().info(f"[{decision}] Critical ID: {pid} | TTC: {ttc:.2f}s", throttle_duration_sec=1.0)

def main(args=None):
    rclpy.init(args=args)
    node = SafetyDecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
import math
import time
import torch
import csv
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict, deque
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseArray, Twist
from std_msgs.msg import String

# --- LSTM Architecture (Must match your training) ---
class PedestrianLSTM(torch.nn.Module):
    def __init__(self, input_size=2, hidden_size=128, num_layers=2):
        super(PedestrianLSTM, self).__init__()
        self.lstm = torch.nn.LSTM(input_size, hidden_size, num_layers, batch_first=True)
        self.fc = torch.nn.Linear(hidden_size, 2)

    def forward(self, x):
        out, _ = self.lstm(x)
        return self.fc(out[:, -1, :])

class SafetyDecisionNode(Node):
    def __init__(self):
        super().__init__('safety_decision_node')

        # === PARAMETERS ===
        self.stop_dist = 15.0       # Proximity safety
        self.stop_ttc = 2.5         # Critical threshold
        self.slow_ttc = 5.0        
        self.lane_width = 10.0      
        self.collision_radius = 1.5 
        self.history_len = 8        # Required for LSTM

        # === MODEL & LOGGING ===
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = PedestrianLSTM().to(self.device)
        try:
            pkg_gazebo_bringup = get_package_share_directory('av_gazebo_bringup')
            model_path = os.path.join(pkg_gazebo_bringup, 'model_training', 'ped_lstm_model.pth')
            print(model_path)
            self.model.load_state_dict(torch.load(model_path, map_location=self.device))
            self.model.eval()
            self.get_logger().info(f"🧠 LSTM Model Loaded from {model_path}")
        except:
            self.get_logger().error("❌ Failed to load LSTM model! Check if ped_lstm_model.pth exists.")

        self.csv_file = open('realtime_comparison.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['time', 'id', 'actual_x', 'actual_y', 'ema_x', 'ema_y', 'lstm_x', 'lstm_y'])

        # === TRACKING STATE ===
        self.tracks = defaultdict(lambda: deque(maxlen=self.history_len))
        self.vel_smooth = defaultdict(lambda: (0.0, 0.0))
        self.last_seen = {}
        self.next_id = 0
        self.alpha = 0.3            
        self.track_timeout = 1.0    
        self.ego_speed = 0.0        

        # === PLOTTING ===
        plt.ion()
        self.fig, self.ax = plt.subplots()
        
        # Subscribers & Publishers
        self.sub_peds = self.create_subscription(PoseArray, '/perception/pedestrians_world', self.ped_callback, 10)
        self.sub_ego = self.create_subscription(Twist, '/ego/cmd_vel', self.ego_callback, 10)
        self.pub = self.create_publisher(String, '/av/decision_state', 10)

    def ego_callback(self, msg: Twist):
        # Forward is Y in your coordinate system
        self.ego_speed = abs(msg.linear.y)

    def ped_callback(self, msg: PoseArray):
        now = self.get_clock().now().nanoseconds / 1e9
        detections = [(pose.position.x, pose.position.y) for pose in msg.poses]
        
        self.ax.clear()
        self.ax.set_xlim(-15, 15); self.ax.set_ylim(-5, 45)
        self.ax.set_title("Real-Time Perception: Actual(B) | EMA(G) | LSTM(R)")

        current_active_ids = []
        for x, y in detections:
            best_id = self.find_best_id(x, y)
            
            if best_id is not None:
                self.tracks[best_id].append([x, y])
                self.last_seen[best_id] = now
            else:
                best_id = self.next_id
                self.next_id += 1
                self.tracks[best_id].append([x, y])
                self.last_seen[best_id] = now
            
            current_active_ids.append(best_id)

            # --- DUAL PREDICTION LOGIC ---
            if len(self.tracks[best_id]) >= 2:
                # 1. EMA Prediction
                last_x, last_y = self.tracks[best_id][-2]
                ema_x = self.alpha * x + (1 - self.alpha) * last_x
                ema_y = self.alpha * y + (1 - self.alpha) * last_y

                # 2. LSTM Prediction
                lstm_x, lstm_y = ema_x, ema_y # Fallback
                if len(self.tracks[best_id]) == self.history_len:
                    input_data = torch.tensor(list(self.tracks[best_id])).float().unsqueeze(0).to(self.device)
                    with torch.no_grad():
                        pred = self.model(input_data)
                        lstm_x, lstm_y = pred[0][0].item(), pred[0][1].item()

                # 3. Visualization & Logging
                self.ax.scatter(x, y, c='blue', label='Actual' if best_id==0 else "")
                self.ax.scatter(ema_x, ema_y, c='green', marker='x')
                self.ax.scatter(lstm_x, lstm_y, c='red', marker='o')
                self.csv_writer.writerow([now, best_id, x, y, ema_x, ema_y, lstm_x, lstm_y])

        plt.pause(0.01)
        self.make_decision(current_active_ids)
        self.cleanup_tracks(now)

    def find_best_id(self, x, y):
        best_id, best_dist = None, 3.0
        for pid, hist in self.tracks.items():
            px, py = hist[-1]
            d = math.hypot(x - px, y - py)
            if d < best_dist:
                best_dist, best_id = d, pid
        return best_id

    def make_decision(self, active_ids):
        decision = "GO"
        min_ttc = float('inf')
        critical_id = -1

        for pid in active_ids:
            x, y = self.tracks[pid][-1]
            dist = math.hypot(x, y)
            
            # Proximity override
            if dist < self.stop_dist:
                decision = "STOP"
                critical_id = pid
                break

            # Velocity-based TTC
            if self.ego_speed > 0.1:
                ttc = (dist - self.collision_radius) / self.ego_speed
                if ttc < min_ttc:
                    min_ttc, critical_id = ttc, pid

        if decision != "STOP":
            if min_ttc < self.stop_ttc: decision = "STOP"
            elif min_ttc < self.slow_ttc: decision = "SLOW"

        self.publish_decision(decision, critical_id, min_ttc)

    def cleanup_tracks(self, now):
        to_delete = [pid for pid, t in self.last_seen.items() if now - t > self.track_timeout]
        for pid in to_delete:
            del self.tracks[pid]
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
        node.csv_file.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
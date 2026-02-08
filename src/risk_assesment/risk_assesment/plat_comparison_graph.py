import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

def generate_dissertation_plots(file_path='results/realtime_comparison.csv'):
    if not os.path.exists(file_path):
        # Fallback to absolute path if local file not found
        file_path = '/home/nk/Music/pedestrian_nav_ai/results/realtime_comparison.csv'
        if not os.path.exists(file_path):
            print(f"Error: {file_path} not found.")
            return

    # Load data using pandas
    df = pd.read_csv(file_path)

    # --- FIX: Convert Pandas Series to NumPy arrays ---
    actual_x = df['actual_x'].values
    actual_y = df['actual_y'].values
    ema_x = df['ema_x'].values
    ema_y = df['ema_y'].values
    lstm_x = df['lstm_x'].values
    lstm_y = df['lstm_y'].values

    # 1. Trajectory Comparison Plot
    plt.figure(figsize=(10, 6))
    plt.plot(actual_x, actual_y, 'b-', label='Actual Path (Ground Truth)', linewidth=2)
    plt.plot(ema_x, ema_y, 'g--', label='EMA (Baseline Smoothing)', alpha=0.8)
    
    # Scaling check for visualization
    is_scaled = False
    if np.mean(lstm_y) < 2.0 and np.mean(actual_y) > 10.0:
        is_scaled = True
        print("⚠️ Scaling LSTM data by 50x for visualization and metrics...")
        plot_lstm_x, plot_lstm_y = lstm_x * 50, lstm_y * 50
        plt.plot(plot_lstm_x, plot_lstm_y, 'r:', label='LSTM (Proposed - Scaled)', alpha=0.8)
    else:
        plot_lstm_x, plot_lstm_y = lstm_x, lstm_y
        plt.plot(plot_lstm_x, plot_lstm_y, 'r:', label='LSTM (Proposed)', alpha=0.8)

    plt.title('Pedestrian Trajectory Prediction: Comparison of Methods')
    plt.xlabel('Lateral Distance X (m)')
    plt.ylabel('Longitudinal Distance Y (m)')
    plt.legend()
    plt.grid(True, linestyle='--', alpha=0.7)
    
    plt.savefig('dissertation_trajectory_fixed.png')
    print("✅ Plot saved as dissertation_trajectory_fixed.png")

    # 2. PERFORMANCE METRIC CALCULATIONS
    # Calculate Euclidean errors at every step
    ema_errors = np.sqrt(np.square(actual_x - ema_x) + np.square(actual_y - ema_y))
    
    # Use scaled LSTM if scaling was triggered for visualization
    calc_lstm_x = lstm_x * 50 if is_scaled else lstm_x
    calc_lstm_y = lstm_y * 50 if is_scaled else lstm_y
    lstm_errors = np.sqrt(np.square(actual_x - calc_lstm_x) + np.square(actual_y - calc_lstm_y))

    # ADE: Average Displacement Error (Mean of all steps)
    ade_ema = np.mean(ema_errors)
    ade_lstm = np.mean(lstm_errors)

    # FDE: Final Displacement Error (Error at the last timestamp)
    fde_ema = ema_errors[-1]
    fde_lstm = lstm_errors[-1]
    
    print("\n" + "="*30)
    print("📊 PERFORMANCE METRICS")
    print("="*30)
    print(f"EMA ADE:  {ade_ema:.4f} meters")
    print(f"EMA FDE:  {fde_ema:.4f} meters")
    print("-" * 30)
    print(f"LSTM ADE: {ade_lstm:.4f} meters {'(Scaled)' if is_scaled else ''}")
    print(f"LSTM FDE: {fde_lstm:.4f} meters {'(Scaled)' if is_scaled else ''}")
    print("="*30)
    
    plt.show()

if __name__ == "__main__":
    generate_dissertation_plots()
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def generate_relative_comparison(bev_file, geo_file, output_img='relative_fusion_comparison.png'):
    # 1. Load the CSV data
    bev_df = pd.read_csv(bev_file)
    geo_df = pd.read_csv(geo_file)

    # 2. Convert to Relative Time (Each starts at 0.0s)
    # Adding .to_numpy() here to prevent the indexing error
    bev_time = (bev_df['Timestamp'] - bev_df['Timestamp'].iloc[0]).to_numpy()
    geo_time = (geo_df['Timestamp'] - geo_df['Timestamp'].iloc[0]).to_numpy()

    # 3. Map Decisions (STOP=0, SLOW=1, GO=2)
    decision_map = {'STOP': 0, 'SLOW': 1, 'GO': 2}
    # Converting the mapped result to a numpy array as well
    bev_state = bev_df['Decision'].map(decision_map).to_numpy()
    geo_state = geo_df['Decision'].map(decision_map).to_numpy()

    # 4. Create the Comparison Figure
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    # BEV Subplot - Using the pre-converted numpy arrays
    ax1.step(bev_time, bev_state, where='post', color='blue', linewidth=2)
    ax1.set_title('BEV Fusion Decision (Relative Timeline)')
    ax1.set_yticks([0, 1, 2])
    ax1.set_yticklabels(['STOP', 'SLOW', 'GO'])
    ax1.grid(True, alpha=0.3)

    # Geometry Subplot - Using the pre-converted numpy arrays
    ax2.step(geo_time, geo_state, where='post', color='orange', linewidth=2, linestyle='--')
    ax2.set_title('Geometry Fusion Decision (Relative Timeline)')
    ax2.set_yticks([0, 1, 2])
    ax2.set_yticklabels(['STOP', 'SLOW', 'GO'])
    ax2.set_xlabel('Relative Time (seconds)')
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_img, dpi=300)
    print(f"✅ Independent relative plot saved as {output_img}")

# Example Usage:
generate_relative_comparison(
    '/home/nk/Music/pedestrian_nav_ai/benchmark_results_BEV.csv', 
    '/home/nk/Music/pedestrian_nav_ai/benchmark_results_gemotry.csv'
)
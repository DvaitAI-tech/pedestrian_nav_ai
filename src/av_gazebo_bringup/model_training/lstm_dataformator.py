import os
import glob
import numpy as np

def prepare_trajectory_data(labels_path, seq_len=8, pred_len=12):
    all_inputs = []
    all_targets = []
    
    # Iterate through each video folder in the labels directory
    video_folders = [f for f in os.listdir(labels_path) if os.path.isdir(os.path.join(labels_path, f))]
    
    for video_folder in video_folders:
        video_path = os.path.join(labels_path, video_folder)
        files = sorted(glob.glob(os.path.join(video_path, "*.txt")))
        
        # Loop through frames to create sequences
        for i in range(len(files) - (seq_len + pred_len)):
            input_seq = [] # Initialize inside the loop to avoid NameError
            
            # Step 1: Build the input sequence (past history)
            valid_sequence = True
            for j in range(i, i + seq_len):
                if os.path.exists(files[j]):
                    with open(files[j], 'r') as f:
                        line = f.readline().split()
                        if line:
                            # YOLO Format: [class_id x_center y_center width height]
                            input_seq.append([float(line[1]), float(line[2])])
                        else:
                            valid_sequence = False
                            break
                else:
                    valid_sequence = False
                    break
            
            # Step 2: Get the target (future position)
            if valid_sequence and len(input_seq) == seq_len:
                target_file = files[i + seq_len + pred_len]
                with open(target_file, 'r') as f:
                    target_line = f.readline().split()
                    if target_line:
                        all_inputs.append(input_seq)
                        all_targets.append([float(target_line[1]), float(target_line[2])])
                    
    return np.array(all_inputs, dtype=np.float32), np.array(all_targets, dtype=np.float32)

# --- EXECUTION ---
labels_path = "/home/nk/.cache/kagglehub/datasets/menhari/ethucyjaad-data-set-with-labels/versions/1/dataset/labels"
print("🛠️  Processing labels from:", labels_path)

X, y = prepare_trajectory_data(labels_path)

# Save the data for training reference
save_path = "processed_pedestrian_data.npz"
np.savez_compressed(save_path, x_train=X, y_train=y)

print(f"✅ Data saved successfully to: {os.path.abspath(save_path)}")
print(f"📊 Total Sequences: {X.shape[0]}")
print(f"📐 Input Shape (History): {X.shape[1:]}")
print(f"🎯 Target Shape (Future): {y.shape[1:]}")
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader
import numpy as np
import time

# --- 1. Custom Dataset for NPZ ---
class TrajectoryDataset(Dataset):
    def __init__(self, data_file):
        data = np.load(data_file)
        self.x = torch.from_numpy(data['x_train']).float()
        self.y = torch.from_numpy(data['y_train']).float()

    def __len__(self):
        return len(self.x)

    def __getitem__(self, idx):
        return self.x[idx], self.y[idx]

# --- 2. LSTM Model Architecture ---
class PedestrianLSTM(nn.Module):
    def __init__(self, input_size=2, hidden_size=128, num_layers=2):
        super(PedestrianLSTM, self).__init__()
        self.lstm = nn.LSTM(input_size, hidden_size, num_layers, batch_first=True)
        self.fc = nn.Linear(hidden_size, 2) # Predicts the next (x, y)

    def forward(self, x):
        # x shape: (batch, seq_len, 2)
        out, _ = self.lstm(x)
        # We only need the last hidden state for the final prediction
        return self.fc(out[:, -1, :])

# --- 3. ADE & FDE Metrics ---
def calculate_metrics(preds, targets):
    # Euclidean distance between prediction and target
    dist = torch.norm(preds - targets, dim=1)
    ade = torch.mean(dist).item()
    fde = ade # Since we are predicting 1 point (the final destination)
    return ade, fde

# --- 4. Main Training Loop ---
def train_model():
    # Settings
    data_path = 'processed_pedestrian_data.npz'
    batch_size = 64
    epochs = 20
    learning_rate = 0.001

    dataset = TrajectoryDataset(data_path)
    dataloader = DataLoader(dataset, batch_size=batch_size, shuffle=True)

    model = PedestrianLSTM()
    criterion = nn.MSELoss()
    optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)

    print(f"🚀 Starting Training on {len(dataset)} samples...")
    for epoch in range(epochs):
        epoch_loss = 0
        all_ade = []

        for batch_x, batch_y in dataloader:
            optimizer.zero_grad()
            preds = model(batch_x)
            loss = criterion(preds, batch_y)
            
            loss.backward()
            optimizer.step()

            ade, _ = calculate_metrics(preds, batch_y)
            all_ade.append(ade)
            epoch_loss += loss.item()

        avg_ade = sum(all_ade) / len(all_ade)
        print(f"Epoch [{epoch+1}/{epochs}] | Loss: {epoch_loss/len(dataloader):.5f} | ADE: {avg_ade:.4f}m")

    # Save the model
    torch.save(model.state_dict(), 'ped_lstm_model.pth')
    print("✅ Model saved as 'ped_lstm_model.pth'")

if __name__ == "__main__":
    train_model()
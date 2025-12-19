import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import TensorDataset, DataLoader
import control as ct
import numpy as np
import matplotlib.pyplot as plt

BATCH_SIZE = 64
LEARNING_RATE = 1e-3
EPOCHS = 20
history = {'loss': []}

A = np.array([[0, 0, 0, 1.0000, 0, 0],
             [0, 0, 0, 0, 1.0000, 0],
              [0, 0, 0, 0, 0, 1.0000],
              [0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0],
              [0, -1.2304, 0, 0, 0, 0]])

B = np.array([[0, 0],
              [0, 0],
              [0, 0],
              [0.0858, 0.0858],
              [0.5810, -0.5810],
              [0, 0]])

C = np.array([[1, 0, 0, 0, 0, 0],
              [0, 1, 0, 0, 0, 0],
              [0, 0, 1, 0, 0, 0]])

D = np.array([[0, 0],
              [0, 0],
              [0, 0]])

sys = ct.ss(A, B, C, D)


def generate_dataset(num_data=20, T=10.0):
    X = []
    Y = []

    t_eval = np.linspace(0, T, 200)

    for k in range(num_data):
        elevation = np.random.uniform(0, 0.5)
        pitch = np.random.uniform(-0.5, 0.5)
        travel = np.random.uniform(0, 0.5)

        x0 = [elevation, pitch, travel, 0, 0, 0]

        t, y = ct.initial_response(sys, T=t_eval, X0=x0)
        for i in range(200):
            X.append(x0)
            Y.append([y[0][i], y[1][i], y[2][i]])

    X = np.array(X, dtype=np.float32)
    Y = np.array(Y, dtype=np.float32)

    return X, Y


X_train, Y_train = generate_dataset()
print("Dataset shape:", X_train.shape, Y_train.shape)
X_train_tensor = torch.tensor(X_train, dtype=torch.float32)
Y_train_tensor = torch.tensor(Y_train, dtype=torch.float32)
train_dataset = TensorDataset(X_train_tensor, Y_train_tensor)
train_loader = DataLoader(train_dataset, batch_size=BATCH_SIZE, shuffle=True)


class DirectModel(nn.Module):
    def __init__(self):
        super(DirectModel, self).__init__()
        self.layer1 = nn.Linear(6, 32)
        self.tanh1 = nn.Tanh()
        self.layer2 = nn.Linear(32, 32)
        self.tanh2 = nn.Tanh()
        # Output shape is (3,)
        self.layer3 = nn.Linear(32, 3)

    def forward(self, x):
        x = self.layer1(x)
        x = self.tanh1(x)
        x = self.layer2(x)
        x = self.tanh2(x)
        x = self.layer3(x)
        return x


direct_model = DirectModel()
optimizer = optim.Adam(direct_model.parameters(), lr=LEARNING_RATE)
criterion = nn.MSELoss()

print("Starting training...")

for epoch in range(EPOCHS):
    direct_model.train()  # Set model to training mode
    running_loss = 0.0

    for batch_idx, (inputs, targets) in enumerate(train_loader):
        # 1. Zero the parameter gradients
        optimizer.zero_grad()

        # 2. Forward pass
        outputs = direct_model(inputs)
        loss = criterion(outputs, targets)

        # 3. Backward pass and optimization
        loss.backward()
        optimizer.step()

        running_loss += loss.item() * inputs.size(0)

    # Calculate average epoch loss
    epoch_loss = running_loss / len(train_loader.dataset)
    history['loss'].append(epoch_loss)

    # Print progress (verbose=1 equivalent)
    print(f"Epoch [{epoch+1}/{EPOCHS}], Loss: {epoch_loss:.4f}")

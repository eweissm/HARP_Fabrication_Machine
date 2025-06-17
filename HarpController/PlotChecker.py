import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


def moving_average(data, window_size):
    return np.convolve(data, np.ones(window_size)/window_size, mode='same')


# Load the CSV file
file_path = 'ControllerData/TrajectoryTracking_3hz_v3.csv'
  # Replace with your actual file path
df = pd.read_csv(file_path, header=None)
t0 = 5
filtered_df = df[df.iloc[:, 0] > t0]

# Extract filtered columns
x = filtered_df.iloc[:, 0]
y_pred = filtered_df.iloc[:, 4]  # Column 7: predicted
y_true = filtered_df.iloc[:, 5]  # Column 8: ground truth

y_pred = moving_average(y_pred, 216)
# Calculate Mean Squared Error manually
mse = np.mean((y_pred - y_true) ** 2)
print(f"Mean Squared Error (MSE): {mse:.6f}")

# Plot the curves
plt.figure(figsize=(10, 6))
plt.plot(x, y_pred, label='Column 7 (Prediction)')
plt.plot(x, y_true, label='Column 8 (Ground Truth)', linestyle='--')

plt.xlabel('Column 1')
plt.ylabel('Values')
plt.title('Prediction vs Ground Truth')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()


# plt.figure(figsize=(10, 6))
# plt.plot(x)
# plt.show()
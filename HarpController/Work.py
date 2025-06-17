import csv
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import interp1d

file_path = "Data/Muscle43_v1_40psi_1000g.csv"
load =1000
g = 9.81
F = g * load / 1000
print(F)

distances = []
times = []

# Read CSV
with open(file_path, 'r') as csvfile:
    reader = csv.reader(csvfile)
    next(reader)  # Skip header

    for row in reader:
        try:
            distance = float(row[3])
            time = float(row[0])
            distances.append(distance)
            times.append(time)
        except (IndexError, ValueError):
            continue

# times_np = np.array(times)
# window_size = 100
# threshold = 2  # Standard deviations from local mean
#
# # Padding to handle borders
# padded_times = np.pad(times_np, (window_size // 2,), mode='edge')
# outliers = np.zeros_like(times_np, dtype=bool)

# # Detect outliers using local window
# for i in range(len(times_np)):
#     window = padded_times[i:i + window_size]
#     local_mean = np.mean(window)
#     local_std = np.std(window)
#
#     if abs(times_np[i] - local_mean) > threshold * local_std:
#         outliers[i] = True
#
# print(f"Detected {np.sum(outliers)} outliers in time data (moving window).")
#
# # Interpolate to fill outliers
# valid_indices = np.where(~outliers)[0]
# valid_times = times_np[~outliers]
# interp_func = interp1d(valid_indices, valid_times, kind='linear', fill_value="extrapolate")
#
# times_filled = times_np.copy()
# times_filled[outliers] = interp_func(np.where(outliers)[0])

# Analysis
if distances:
    max_dist = max(distances)
    min_dist = min(distances)
    print(f"Max distance: {max_dist}")
    print(f"Min distance: {min_dist}")
    print(f"Difference: {max_dist - min_dist}")
    print(f"Work: {F * (max_dist - min_dist) / 1000}")
else:
    print("No valid distances found.")

# Plot results
plt.plot(times, distances, linewidth=2)
plt.show()

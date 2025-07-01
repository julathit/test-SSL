import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Sample noisy data
np.random.seed(42)
x_data = np.linspace(0, 10, 50)
y_data = np.sin(x_data) + np.random.normal(0, 0.2, 50)

# Create a Pandas Series
series = pd.Series(y_data)

# Apply moving average with a window of 5
window_size = 5
y_smoothed_ma = series.rolling(window=window_size, center=True).mean()

# Plotting
plt.figure(figsize=(10, 6))
plt.plot(x_data, y_data, 'b.', label='Noisy Data')
plt.plot(x_data, y_smoothed_ma.to_numpy(), 'r-', label=f'Moving Average (window={window_size})')
plt.xlabel('X')
plt.ylabel('Y')
plt.legend()
plt.title('Moving Average Smoothing')
plt.grid(True)
plt.show()
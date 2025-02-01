import numpy as np
import pandas as pd
from scipy.optimize import curve_fit
from sklearn.model_selection import KFold
from sklearn.metrics import mean_absolute_error, mean_squared_error, r2_score

# Load the data from the CSV file
df = pd.read_csv('motor-data.csv')

x = df['Torque [Nm]'].values
y = df['Speed [rpm]'].values
z = df['DC Current [A]'].values

# Define the function to fit the data
def func(xy, a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p, q, r, s, t, u, v, w):
    x, y = xy
    return (a + b*x + c*y + d*(x**2) + e*(y**2) + f*x*y + g*(x**3) + h*(y**3) +
            i*(x**2)*y + j*x*(y**2) + k*(x**4) + l*(y**4) + m*(x**3)*y +
            n*(x**2)*(y**2) + o*x*(y**3) + p*(x**5) + q*(y**5) + r*(x**4)*y +
            s*(x**3)*(y**2) + t*(x**2)*(y**3) + u*x*(y**4) + v*(x**6) + w*(y**6))

# Define the prediction function
def predict(x, y, coefficients):
    return func((x, y), *coefficients)

# Initialize k-fold cross-validation
kf = KFold(n_splits=5, shuffle=True, random_state=42)

mae_list = []
mse_list = []
r2_list = []

# Perform k-fold cross-validation
for train_idx, test_idx in kf.split(x):
    # Split the data into training and testing
    x_train, x_test = x[train_idx], x[test_idx]
    y_train, y_test = y[train_idx], y[test_idx]
    z_train, z_test = z[train_idx], z[test_idx]

    # Fit the curve on the training data
    popt, _ = curve_fit(func, (x_train, y_train), z_train)

    # Predict on the testing data
    z_pred = predict(x_test, y_test, popt)

    # Compute metrics for this fold
    mae_list.append(mean_absolute_error(z_test, z_pred))
    mse_list.append(mean_squared_error(z_test, z_pred))
    r2_list.append(r2_score(z_test, z_pred))

# Calculate average metrics across all folds
mae = np.mean(mae_list)
mse = np.mean(mse_list)
r2_mean = np.mean(r2_list)

# Compute percentage metrics
mean_current = np.mean(z)
mae_percentage = (mae / mean_current) * 100
mse_percentage = (mse / mean_current) * 100

# Print accuracy results
print(f"Cross-Validation MAE: {mae:.4f}")
print(f"Cross-Validation MAE (Percentage): {mae_percentage:.2f}%")
print(f"Cross-Validation MSE: {mse:.4f}")
print(f"Cross-Validation MSE (Percentage): {mse_percentage:.2f}%")
print(f"R² Score (Mean): {r2_mean:.4f}")

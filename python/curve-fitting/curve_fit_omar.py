import numpy as np
import pandas as pd

# Get the data from the CSV file
df = pd.read_csv('motor-data.csv')

x = df['Torque [Nm]'].values
y = df['Speed [rpm]'].values
z = df['DC Current [A]'].values

# Generate polynomial terms for the design matrix
def generate_terms(x, y):
    """Generate all polynomial terms up to x^6 and y^6."""
    return np.column_stack([
        np.ones_like(x),  # Constant term
        x, y,
        x**2, y**2, x*y,
        x**3, y**3, x**2 * y, x * y**2,
        x**4, y**4, x**3 * y, x**2 * y**2, x * y**3,
        x**5, y**5, x**4 * y, x**3 * y**2, x**2 * y**3, x * y**4,
        x**6, y**6
    ])

# Function to predict values using the fitted model
def predict(x, y, coeffs):
    """Predict DC Current [A] using the fitted coefficients."""
    terms = generate_terms(x, y)
    return terms @ coeffs

# LOOCV implementation
mae_total = 0
mse_total = 0
mean_current = np.mean(z)
for i in range(len(x)):
    # Leave out the i-th data point
    x_train = np.delete(x, i)
    y_train = np.delete(y, i)
    z_train = np.delete(z, i)

    # Construct the design matrix for the training set
    X_train = generate_terms(x_train, y_train)

    # Solve for coefficients using linear least squares
    coefficients, residuals, rank, singular_values = np.linalg.lstsq(X_train, z_train, rcond=None)

    # Predict the left-out data point
    x_test, y_test, z_test = x[i], y[i], z[i]
    z_pred = predict(x_test, y_test, coefficients)

    # Calculate the errors for this prediction
    mae_total += np.abs(z_test - z_pred)
    mse_total += (z_test - z_pred)**2

# Calculate overall MAE and MSE from LOOCV
mae = mae_total / len(x)
mse = mse_total / len(x)

# Calculate percentage errors
mae_percentage = (mae / mean_current) * 100
mse_percentage = (mse / mean_current) * 100

# Print results
print(f"LOOCV MAE: {mae.item():.4f}")  # Use .item() to access the scalar value
print(f"LOOCV MAE (Percentage): {mae_percentage.item():.2f}%")
print(f"LOOCV MSE: {mse.item():.4f}")  # Use .item() to access the scalar value
print(f"LOOCV MSE (Percentage): {mse_percentage.item():.2f}%")

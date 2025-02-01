import numpy as np
import pandas as pd
from sklearn.model_selection import KFold
from sklearn.metrics import r2_score
from sklearn.linear_model import Ridge

# Get the data from the CSV file
df = pd.read_csv('motor-data.csv')

x = df['Torque [Nm]'].values
y = df['Speed [rpm]'].values
z = df['DC Current [A]'].values

# Normalize input data to improve numerical stability
x_mean, x_std = np.mean(x), np.std(x)
y_mean, y_std = np.mean(y), np.std(y)

x_norm = (x - x_mean) / x_std
y_norm = (y - y_mean) / y_std

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

# Implement k-fold cross-validation
kf = KFold(n_splits=5, shuffle=True, random_state=42)
mae_total = 0
mse_total = 0
r2_scores = []
coefficients_final = None
mean_current = np.mean(z)

for train_index, test_index in kf.split(x_norm):
    # Split data into training and testing
    x_train, x_test = x_norm[train_index], x_norm[test_index]
    y_train, y_test = y_norm[train_index], y_norm[test_index]
    z_train, z_test = z[train_index], z[test_index]

    # Construct the design matrix for the training set
    X_train = generate_terms(x_train, y_train)

    # Ridge Regression to avoid overfitting (set alpha as regularization strength)
    model = Ridge(alpha=1e-3, fit_intercept=False)  # No intercept; terms include constant
    model.fit(X_train, z_train)
    coefficients = model.coef_

    # Predict on the test set
    X_test = generate_terms(x_test, y_test)
    z_pred = X_test @ coefficients

    # Store coefficients for final use (average over folds)
    if coefficients_final is None:
        coefficients_final = coefficients
    else:
        coefficients_final += coefficients

    # Calculate errors and R² score
    mae_total += np.mean(np.abs(z_test - z_pred))
    mse_total += np.mean((z_test - z_pred)**2)
    r2_scores.append(r2_score(z_test, z_pred))

# Average coefficients across folds
coefficients_final /= kf.get_n_splits()

# Calculate overall MAE, MSE, and R²
mae = mae_total / kf.get_n_splits()
mse = mse_total / kf.get_n_splits()
r2_mean = np.mean(r2_scores)

# Calculate percentage errors
mae_percentage = (mae / mean_current) * 100
mse_percentage = (mse / mean_current) * 100

# Print results
print(f"Final Coefficients: {coefficients_final}")
print(f"Cross-Validation MAE: {mae:.4f}")
print(f"Cross-Validation MAE (Percentage): {mae_percentage:.2f}%")
print(f"Cross-Validation MSE: {mse:.4f}")
print(f"Cross-Validation MSE (Percentage): {mse_percentage:.2f}%")
print(f"R² Score (Mean): {r2_mean:.4f}")

# Single prediction: Define a single torque and speed value
test_torque = 35.0  # Example Torque [Nm]
test_speed = 1000.0  # Example Speed [rpm]
print("x mean:", x_mean)
print("x_std:", x_std)
print("y_mean:", y_mean)
print("y_std:", y_std)
# Normalize the test inputs
test_torque_norm = (test_torque - x_mean) / x_std
test_speed_norm = (test_speed - y_mean) / y_std

# Use the final coefficients to predict the current for this test input
predicted_current = predict(test_torque_norm, test_speed_norm, coefficients_final)

# Print the prediction
print(f"\nPredicted DC Current for Torque: {test_torque} Nm, Speed: {test_speed} rpm is: {predicted_current} A")

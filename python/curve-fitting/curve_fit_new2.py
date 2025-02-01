import numpy as np
import pandas as pd
from sklearn.model_selection import KFold
from sklearn.linear_model import Ridge
from sklearn.metrics import r2_score, mean_absolute_error, mean_squared_error

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
    """Generate all polynomial terms up to x^6 and y^6, including extra interaction terms."""
    return np.column_stack([
        np.ones_like(x),  # Constant term
        x, y,
        x**2, y**2, x*y,
        x**3, y**3, x**2 * y, x * y**2,
        x**4, y**4, x**3 * y, x**2 * y**2, x * y**3,
        x**5, y**5, x**4 * y, x**3 * y**2, x**2 * y**3, x * y**4,
        x**6, y**6,
        x**3 * y**3, x**4 * y**2, x**2 * y**4  # New interaction terms
    ])

# Function to predict values using the fitted model
def predict(x, y, coeffs):
    """Predict DC Current [A] using the fitted coefficients."""
    terms = generate_terms(x, y)
    return terms @ coeffs

# Grid search to find the best alpha for Ridge Regression
alphas = [1e-4, 1e-3, 1e-2, 1e-1, 1]
best_alpha = None
best_mae = float('inf')

kf_alpha_search = KFold(n_splits=5, shuffle=True, random_state=42)
for alpha in alphas:
    mae_total_alpha = 0
    for train_index, test_index in kf_alpha_search.split(x_norm):
        # Split data into training and testing
        x_train, x_test = x_norm[train_index], x_norm[test_index]
        y_train, y_test = y_norm[train_index], y_norm[test_index]
        z_train, z_test = z[train_index], z[test_index]

        # Construct the design matrix for the training set
        X_train = generate_terms(x_train, y_train)

        # Ridge regression model
        model = Ridge(alpha=alpha, fit_intercept=False)
        model.fit(X_train, z_train)

        # Predict on the test set
        X_test = generate_terms(x_test, y_test)
        z_pred = X_test @ model.coef_

        # Calculate MAE for this fold
        mae_total_alpha += mean_absolute_error(z_test, z_pred)

    # Average MAE for this alpha
    mae_alpha = mae_total_alpha / kf_alpha_search.get_n_splits()
    if mae_alpha < best_mae:
        best_mae = mae_alpha
        best_alpha = alpha

# Implement 10-fold cross-validation using the best alpha
kf = KFold(n_splits=10, shuffle=True, random_state=42)
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

    # Ridge regression with the best alpha
    model = Ridge(alpha=best_alpha, fit_intercept=False)
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
    mae_total += mean_absolute_error(z_test, z_pred)
    mse_total += mean_squared_error(z_test, z_pred)
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
print(f"Best Alpha: {best_alpha}")
print(f"Final Coefficients: {coefficients_final}")
print(f"Cross-Validation MAE: {mae:.4f}")
print(f"Cross-Validation MAE (Percentage): {mae_percentage:.2f}%")
print(f"Cross-Validation MSE: {mse:.4f}")
print(f"Cross-Validation MSE (Percentage): {mse_percentage:.2f}%")
print(f"R² Score (Mean): {r2_mean:.4f}")

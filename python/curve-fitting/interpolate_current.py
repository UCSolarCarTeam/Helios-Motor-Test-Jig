import pandas as pd
import numpy as np
from scipy.interpolate import griddata
import matplotlib.pyplot as plt
from sklearn.metrics import mean_absolute_error, mean_squared_error
from mpl_toolkits.mplot3d import Axes3D

# Step 1: Load the data from CSV
file_path = "C:/Users/Omar Hassan/Desktop/Solarcar/Helios-Motor-Test-Jig/python/curve-fitting/motor-data.csv"  # Replace with your CSV file path
data = pd.read_csv(file_path)

# Step 2: Extract the columns
torque = data["Torque [Nm]"]
speed = data["Speed [rpm]"]
current = data["DC Current [A]"]

# Step 3: Handle duplicates by averaging the current values for each unique torque-speed combination
data_cleaned = data.groupby(["Torque [Nm]", "Speed [rpm]"], as_index=False).agg({"DC Current [A]": "mean"})
print("Shape of clean data:", data_cleaned.shape)

# Step 4: Prepare data for interpolation from the cleaned dataset
torque_unique = np.unique(data_cleaned["Torque [Nm]"])
speed_unique = np.unique(data_cleaned["Speed [rpm]"])

# Create a grid for Torque and Speed
grid_torque, grid_speed = np.meshgrid(torque_unique, speed_unique)

# Step 4: Update Interpolation Function with Backup
def interpolate_current_with_backup(points, values, query_torque, query_speed):
    """
    Interpolate current with cubic, fallback to linear, and nearest methods.
    """
    query_points = np.array([query_torque, query_speed]).T

    # Primary method: Cubic interpolation
    result = griddata(points, values, query_points, method='linear')

    # Backup 1: Linear interpolation if cubic fails
    if np.isnan(result).any():
        result[np.isnan(result)] = griddata(points, values, query_points[np.isnan(result)], method='cubic')

    # Backup 2: Nearest-neighbor interpolation if linear fails
    if np.isnan(result).any():
        result[np.isnan(result)] = griddata(points, values, query_points[np.isnan(result)], method='nearest')

    return result



# Step 5: Modify LOOCV to Test Before and After Backup
def loocv_error_with_backup(data, use_backup=False):
    """
    Perform Leave-One-Out Cross-Validation with optional backup interpolation.
    """
    actual_values = []
    predicted_values = []
    nan_count = 0  # Track the number of NaN predictions

    for i in range(len(data)):
        # Remove the i-th data point
        train_data = data.drop(i)
        test_point = data.iloc[i]

        # Prepare training data
        train_torque = train_data["Torque [Nm]"]
        train_speed = train_data["Speed [rpm]"]
        train_current = train_data["DC Current [A]"]

        # Interpolate
        points = np.array([train_torque, train_speed]).T
        values = train_current.values
        query_point = np.array([[test_point["Torque [Nm]"], test_point["Speed [rpm]"]]])

        # Debugging: Print the query point and check the neighborhood
        print(f"Interpolating for point: Torque = {test_point['Torque [Nm]']}, Speed = {test_point['Speed [rpm]']}")
        print(f"Training data range: Torque = {train_torque.min()} to {train_torque.max()}, Speed = {train_speed.min()} to {train_speed.max()}")

        # Choose interpolation method
        if use_backup:
            predicted_current = interpolate_current_with_backup(
                points, values, query_point[:, 0], query_point[:, 1]
            )
        else:
            predicted_current = griddata(points, values, query_point, method='linear')

        # Check if interpolation failed
        if np.isnan(predicted_current):
            print(f"NaN result for point: Torque = {test_point['Torque [Nm]']}, Speed = {test_point['Speed [rpm]']}")
            nan_count += 1
            continue  # Skip this point

        # Append actual and predicted values
        actual_values.append(test_point["DC Current [A]"])
        predicted_values.append(predicted_current[0])  # griddata returns an array

    # Print warning if NaNs occurred
    if nan_count > 0:
        print(f"Warning: {nan_count} points resulted in NaN predictions and were skipped.")

    # Calculate error metrics
    mae = mean_absolute_error(actual_values, predicted_values)
    mse = mean_squared_error(actual_values, predicted_values)
    return mae, mse, nan_count



# Step 6: Compare Pre-Backup and Post-Backup Results
print("Pre-Backup Results:")
mae_pre, mse_pre, nan_pre = loocv_error_with_backup(data_cleaned, use_backup=False)
print(f"MAE: {mae_pre}, MSE: {mse_pre}, NaN Predictions: {nan_pre}")

print("\nPost-Backup Results:")
mae_post, mse_post, nan_post = loocv_error_with_backup(data_cleaned, use_backup=True)
print(f"MAE: {mae_post}, MSE: {mse_post}, NaN Predictions: {nan_post}")

# Calculate percentage improvements
mean_current = data_cleaned["DC Current [A]"].mean()
print(f"\nError as Percentage of Mean Current:")
print(f"Pre-Backup MAE: {(mae_pre / mean_current) * 100:.2f}%")
print(f"Pre-Backup MSE: {(mse_pre / mean_current) * 100:.2f}%")
print(f"Post-Backup MAE: {(mae_post / mean_current) * 100:.2f}%")
print(f"Post-Backup MSE: {(mse_post / mean_current) * 100:.2f}%")


# Step 6: Visualization
# Interpolated grid for the contour plot
grid_current = griddata(
    np.array([data_cleaned["Torque [Nm]"], data_cleaned["Speed [rpm]"]]).T,
    data_cleaned["DC Current [A]"].values,
    (grid_torque, grid_speed),
    method='cubic'
)

# Contour plot
plt.figure(figsize=(8, 6))
plt.contourf(grid_torque, grid_speed, grid_current, cmap='viridis', levels=20)
plt.colorbar(label="DC Current [A]")
plt.scatter(data_cleaned["Torque [Nm]"], data_cleaned["Speed [rpm]"], c='red', label="Data Points")
plt.xlabel("Torque [Nm]")
plt.ylabel("Speed [rpm]")
plt.title("Interpolated Current Surface (Contour Plot)")
plt.legend()
plt.show()

# 3D Plot
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')

# Plot the surface
ax.plot_surface(grid_torque, grid_speed, grid_current, cmap='viridis', alpha=0.8)

# Scatter original data points for reference
ax.scatter(data_cleaned["Torque [Nm]"], data_cleaned["Speed [rpm]"], 
           data_cleaned["DC Current [A]"], color='red', label='Data Points')

# Set labels
ax.set_xlabel("Torque [Nm]")
ax.set_ylabel("Speed [rpm]")
ax.set_zlabel("DC Current [A]")
ax.set_title("3D Plot of Torque, Speed, and Current")
ax.legend()
plt.show()



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

    # Backup 2: Nearest-neighbor interpolation if linear fails
    if np.isnan(result).any():
        result[np.isnan(result)] = griddata(points, values, query_points[np.isnan(result)], method='nearest')

    return result



# Step 5: Modify LOOCV to Test Before and After Backup
def loocv_error_with_backup(data):
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



        predicted_current = interpolate_current_with_backup(
                points, values, query_point[:, 0], query_point[:, 1]
            )

        # Check if interpolation failed
        if np.isnan(predicted_current):
            nan_count += 1
            continue  # Skip this point

        # Append actual and predicted values
        actual_values.append(test_point["DC Current [A]"])
        predicted_values.append(predicted_current[0])  # griddata returns an array

    # Calculate error metrics
    mae = mean_absolute_error(actual_values, predicted_values)
    mse = mean_squared_error(actual_values, predicted_values)
    return mae, mse, nan_count


mae_post, mse_post, nan_post = loocv_error_with_backup(data_cleaned)
print(f"MAE: {mae_post}, MSE: {mse_post}, NaN Predictions: {nan_post}")

# Calculate percentage improvements
mean_current = data_cleaned["DC Current [A]"].mean()

print(f"MAE: {(mae_post / mean_current) * 100:.2f}%")
print(f"MSE: {(mse_post / mean_current) * 100:.2f}%")









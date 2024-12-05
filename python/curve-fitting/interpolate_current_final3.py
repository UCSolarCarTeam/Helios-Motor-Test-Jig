import pandas as pd
import numpy as np
from sklearn.metrics import mean_absolute_error, mean_squared_error

def loocv_error_with_bilinear(data):
    """
    Perform Leave-One-Out Cross-Validation using bilinear interpolation, falling back to nearest-neighbor interpolation if needed.
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

        # Perform bilinear interpolation for the test point
        predicted_current = bilinear_interpolate(test_point["Torque [Nm]"], test_point["Speed [rpm]"], train_data)

        # If bilinear interpolation fails, fallback to nearest-neighbor interpolation
        if predicted_current is None:
            predicted_current = nearest_neighbor_interpolate(test_point["Torque [Nm]"], test_point["Speed [rpm]"], train_data["DC Current [A]"], train_data)
        if predicted_current is None:
                nan_count += 1  # This point failed completely
                continue  # Skip this point if both methods fail

        # Append actual and predicted values
        actual_values.append(test_point["DC Current [A]"])
        predicted_values.append(predicted_current)

    # Calculate error metrics
    mae = mean_absolute_error(actual_values, predicted_values)
    mse = mean_squared_error(actual_values, predicted_values)
    
    return mae, mse, nan_count

# Function for linear interpolation
def linear_interpolate(x, x1, x2, y1, y2):
    return y1 + (x - x1) * (y2 - y1) / (x2 - x1)

# Function to find the closest match for a given value in a column
def closest(value, column, data):
    closest_idx = (data[column] - value).abs().idxmin()
    return data.loc[closest_idx]

# Function to find the closest lower and upper points for torque and speed
def find_surrounding_points(torque, speed, data):
    lower_torque_points = data[data['Torque [Nm]'] < torque].sort_values(by='Torque [Nm]', ascending=False)
    higher_torque_points = data[data['Torque [Nm]'] > torque].sort_values(by='Torque [Nm]', ascending=True)
    lower_speed_points = data[data['Speed [rpm]'] < speed].sort_values(by='Speed [rpm]', ascending=False)
    higher_speed_points = data[data['Speed [rpm]'] > speed].sort_values(by='Speed [rpm]', ascending=True)
    
    if lower_torque_points.empty or higher_torque_points.empty or lower_speed_points.empty or higher_speed_points.empty:
        return None
    
    t1 = lower_torque_points.iloc[0]['Torque [Nm]']
    t2 = higher_torque_points.iloc[0]['Torque [Nm]']
    s1 = lower_speed_points.iloc[0]['Speed [rpm]']
    s2 = higher_speed_points.iloc[0]['Speed [rpm]']
    
    if t1 == t2 or s1 == s2:
        return None
    
    return t1, t2, s1, s2

# Function for bilinear interpolation
def bilinear_interpolate(torque, speed, data):
    surrounding_points = find_surrounding_points(torque, speed, data)
    
    if surrounding_points is None:
        print(f"Error: Could not find valid surrounding points for (Torque: {torque}, Speed: {speed}).")
        return None
    
    t1, t2, s1, s2 = surrounding_points
    
    try:
        c1 = closest(t1, 'Torque [Nm]', data[data['Speed [rpm]'] == s1])
        c2 = closest(t2, 'Torque [Nm]', data[data['Speed [rpm]'] == s1])
        c3 = closest(t1, 'Torque [Nm]', data[data['Speed [rpm]'] == s2])
        c4 = closest(t2, 'Torque [Nm]', data[data['Speed [rpm]'] == s2])
        
        if len(c1) == 0 or len(c2) == 0 or len(c3) == 0 or len(c4) == 0:
            return None
        
    except IndexError:
        return None
    
    interpolated_c1 = linear_interpolate(torque, t1, t2, c1, c2)
    interpolated_c2 = linear_interpolate(torque, t1, t2, c3, c4)
    interpolated_current = linear_interpolate(speed, s1, s2, interpolated_c1, interpolated_c2)
    
    return interpolated_current.iloc[1]

# Nearest-neighbor interpolation function
def nearest_neighbor_interpolate(query_torque, query_speed, current, data):
    """
    Perform nearest-neighbor interpolation.
    """
    # Calculate distances to all points in the data
    distances = np.sqrt((data["Torque [Nm]"] - query_torque) ** 2 + (data["Speed [rpm]"] - query_speed) ** 2)
    nearest_idx = np.argmin(distances)
    
    return current.iloc[nearest_idx]

# Read CSV data
file_path = "C:/Users/Omar Hassan/Desktop/Solarcar/Helios-Motor-Test-Jig/python/curve-fitting/motor-data.csv"  # Replace with your CSV file path
data = pd.read_csv(file_path)

# Example usage for interpolation
torque_value = 62  # Replace with your desired torque value
speed_value = 950  # Replace with your desired speed value
interpolated_current = bilinear_interpolate(torque_value, speed_value, data)

if interpolated_current is not None:
    print(f"Interpolated Current: {interpolated_current} A")
else:
    print("Interpolation failed.")

# Example usage with your data
mae_post, mse_post, nan_post = loocv_error_with_bilinear(data)
print(f"MAE: {mae_post}, MSE: {mse_post}, NaN Predictions: {nan_post}")

# Calculate percentage improvements
mean_current = data["DC Current [A]"].mean()

print(f"MAE: {(mae_post / mean_current) * 100:.2f}%")
print(f"MSE: {(mse_post / mean_current) * 100:.2f}%")

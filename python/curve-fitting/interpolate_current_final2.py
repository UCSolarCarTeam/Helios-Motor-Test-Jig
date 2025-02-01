import pandas as pd

# Function for linear interpolation
def linear_interpolate(x, x1, x2, y1, y2):
    return y1 + (x - x1) * (y2 - y1) / (x2 - x1)

# Function to find the closest lower and upper points for torque and speed
def find_surrounding_points(torque, speed, data):
    # Find the closest lower and upper values for torque
    lower_torque_points = data[data['Torque [Nm]'] < torque].sort_values(by='Torque [Nm]', ascending=False)
    higher_torque_points = data[data['Torque [Nm]'] > torque].sort_values(by='Torque [Nm]', ascending=True)
    
    # Find the closest lower and upper values for speed
    lower_speed_points = data[data['Speed [rpm]'] < speed].sort_values(by='Speed [rpm]', ascending=False)
    higher_speed_points = data[data['Speed [rpm]'] > speed].sort_values(by='Speed [rpm]', ascending=True)
    
    # If we can't find any surrounding points, return None
    if lower_torque_points.empty or higher_torque_points.empty or lower_speed_points.empty or higher_speed_points.empty:
        return None
    
    # Ensure that we have distinct points for both torque and speed
    t1 = lower_torque_points.iloc[0]['Torque [Nm]']
    t2 = higher_torque_points.iloc[0]['Torque [Nm]']
    s1 = lower_speed_points.iloc[0]['Speed [rpm]']
    s2 = higher_speed_points.iloc[0]['Speed [rpm]']
    
    # If we end up with the same point for both lower and higher values, adjust the logic
    if t1 == t2:  # If the lower and higher torque are the same, find the next available distinct point
        if len(lower_torque_points) > 1:
            t1 = lower_torque_points.iloc[1]['Torque [Nm]']
        elif len(higher_torque_points) > 1:
            t2 = higher_torque_points.iloc[1]['Torque [Nm]']
        else:
            return None  # No surrounding points can be found if they are the same
    
    if s1 == s2:  # If the lower and higher speed are the same, find the next available distinct point
        if len(lower_speed_points) > 1:
            s1 = lower_speed_points.iloc[1]['Speed [rpm]']
        elif len(higher_speed_points) > 1:
            s2 = higher_speed_points.iloc[1]['Speed [rpm]']
        else:
            return None  # No surrounding points can be found if they are the same
    
    return t1, t2, s1, s2

# Function for bilinear interpolation
def bilinear_interpolate(torque, speed, data):
    # Find surrounding points
    surrounding_points = find_surrounding_points(torque, speed, data)
    
    if surrounding_points is None:
        print(f"Error: Could not find valid surrounding points for (Torque: {torque}, Speed: {speed}).")
        return None
    
    t1, t2, s1, s2 = surrounding_points
    print(f"Surrounding points for interpolation: t1={t1}, t2={t2}, s1={s1}, s2={s2}")
    
    # Extract the corresponding current values, ensuring that points exist
    try:
        c1 = data[(data['Torque [Nm]'] == t1)]
        c2 = data[(data['Torque [Nm]'] == t2)]
        c3 = data[(data['Speed [rpm]'] == s1)]
        c4 = data[(data['Speed [rpm]'] == s2)]
        
        # Print the number of matches for debugging
        print(f"Matches for c1: {len(c1)}, Matches for c2: {len(c2)}, Matches for c3: {len(c3)}, Matches for c4: {len(c4)}")
        
        # Check if we found the expected current values
        if len(c1) == 0 or len(c2) == 0 or len(c3) == 0 or len(c4) == 0:
            print(f"Error: Missing current values for one or more of the surrounding points.")
            return None
        
        # Extract current values
        c1 = c1['DC Current [A]'].iloc[0]
        c2 = c2['DC Current [A]'].iloc[0]
        c3 = c3['DC Current [A]'].iloc[0]
        c4 = c4['DC Current [A]'].iloc[0]
        
    except IndexError:
        print(f"Error: Could not find all surrounding current values for (Torque: {torque}, Speed: {speed}).")
        return None
    
    # Perform interpolation if all surrounding points are found
    interpolated_c1 = linear_interpolate(torque, t1, t2, c1, c2)
    interpolated_c2 = linear_interpolate(torque, t1, t2, c3, c4)
    interpolated_current = linear_interpolate(speed, s1, s2, interpolated_c1, interpolated_c2)
    
    return interpolated_current

# Read CSV data
file_path = "C:/Users/Omar Hassan/Desktop/Solarcar/Helios-Motor-Test-Jig/python/curve-fitting/motor-data.csv" # Replace with your CSV file path
data = pd.read_csv(file_path)

# Example usage for interpolation
torque_value = 62  # Replace with your desired torque value
speed_value = 950  # Replace with your desired speed value
interpolated_current = bilinear_interpolate(torque_value, speed_value, data)

if interpolated_current is not None:
    print(f"Interpolated Current: {interpolated_current} A")
else:
    print("Interpolation failed.")










import numpy as np
import pandas as pd

from scipy.optimize import curve_fit

# Get the data from the csv file
df = pd.read_csv('motor-data.csv')

x = df['Torque [Nm]']
y = df['DC current [A]']
z = df['Speed [rpm]']

data = np.array([x, y, z])

def func(xy, a, b, c, d, e, f): 
    x, y = xy 
    return a + b*x + c*y + d*x**2 + e*y**2 + f*x*y

# Perform curve fitting 
popt, pcov = curve_fit(func, (x, y), z) 
  
# Print optimized parameters 
print(popt)

# Predict the values
z_pred = func((x, y), *popt)

# Calculate the error
error = np.mean(np.abs(z - z_pred))
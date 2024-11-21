# Curve fitting using scipy.optimize.curve_fit
# Created by: Dominic Choi

import numpy as np
import pandas as pd

from scipy.optimize import curve_fit

# Get the data from the csv file
df = pd.read_csv('motor-data.csv')

x = df['Torque [Nm]']
y = df['Speed [rpm]']

z = df['DC Current [A]']

data = np.array([x, y, z])

print(data)

# Define the function to fit the data
def func(xy, a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p, q, r, s, t, u, v, w): 
    x, y = xy 
    return a + b*x + c*y + d*(x**2) + e*(y**2) + f*x*y + g*(x**3) + h*(y**3) + i*(x**2)*y + j*x*(y**2) + k*(x**4) + l*(y**4) + m*(x**3)*y + n*(x**2)*(y**2) + o*x*(y**3) + p*(x**5) + q*(y**5) + r*(x**4)*y + s*(x**3)*(y**2) + t*(x**2)*(y**3) + u*x*(y**4) + v*(x**6) + w*(y**6)

# Perform curve fitting 
popt, pcov = curve_fit(func, (x, y), z) 
  
# # Print optimized parameters 
print(popt)

# Predicted values
def predict(x, y):
    z = popt[0] + popt[1]*x + popt[2]*y + popt[3]*(x**2) + popt[4]*(y**2) + popt[5]*x*y + popt[6]*(x**3) + popt[7]*(y**3) + popt[8]*(x**2)*y + popt[9]*x*(y**2) + popt[10]*(x**4) + popt[11]*(y**4) + popt[12]*(x**3)*y + popt[13]*(x**2)*(y**2) + popt[14]*x*(y**3) + popt[15]*(x**5) + popt[16]*(y**5) + popt[17]*(x**4)*y + popt[18]*(x**3)*(y**2) + popt[19]*(x**2)*(y**3) + popt[20]*x*(y**4) + popt[21]*(x**6) + popt[22]*(y**6)
    return z

zpred = []
for i in range(len(x)):
    zpred.append(predict(x[i], y[i]))

# Calculate the error
error = (sum(abs(z - zpred))/len(z))
max_error = max((z - zpred))
min_error = min((z - zpred))

print(error, "% Error")
print(max_error, "Max Error")
print(min_error, "Min Error")
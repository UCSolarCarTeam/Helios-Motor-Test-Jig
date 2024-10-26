import numpy as np
import pandas as pd

from scipy.optimize import curve_fit

# Get the data from the csv file
df = pd.read_csv('motor-data.csv')

x = df['Torque [Nm]']
y = df['Speed [rpm]']

z = df['DC Current [A]']

data = np.array([x, y, z])

def func(xy, a, b, c, d, e, f): 
    x, y = xy 
    return a + b*x + c*y + d*x**2 + e*y**2 + f*x*y

# Perform curve fitting 
popt, pcov = curve_fit(func, (x, y), z) 
  
# # Print optimized parameters 
# print(popt)

# Predicted values
def predict(x, y):
    z = 1.68577821 + (-7.08012537*(10**-2))*x - (1.70815244*(10**-4))*y + (1.29804366*(10**-3))*(x**2) + (1.33784126*10**-5)*(y**2) + (2.45372892*(10**-3))*x*y
    return z

zpred = []
for i in range(len(x)):
    zpred.append(predict(x[i], y[i]))

# Calculate the error
error = (sum(abs(z - zpred))/len(z))

print(error, "% Error")
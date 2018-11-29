import matplotlib.pyplot as plt
import numpy as np


alpha_values = []
Cl_values = []
Cd_values = []
Cm_values = []

with open('e423_data.txt') as f:
    for i in xrange(12):
        f.next()
    for line in f:
        values = line.split()
        alpha = float(values[0])
        Cl = float(values[1])
        Cd = float(values[2])
        Cm = float(values[4])

        alpha_values.append(alpha)
        Cl_values.append(Cl)
        Cd_values.append(Cd)
        Cm_values.append(Cm)

desired_alpha_value = 0.0

alpha_values_arr = np.array(alpha_values)


def get_index(desired_alpha_value):
    bool_arr =  np.isclose(alpha_values_arr, desired_alpha_value * np.ones(np.size(alpha_values_arr)))
    index_list = np.where(bool_arr == True)
    print alpha_values[index_list[0][0]]
    return index_list[0][0]

def get_aerdynamic_coefficient(x,y):
    x_rad = x * np.pi/180.
    m,b = np.polyfit(x_rad,y,1)
    return m, b

def abline(slope, intercept):
    """Plot a line from slope and intercept"""
    axes = plt.gca()
    x_vals = np.array(axes.get_xlim())
    y_vals = intercept + slope * x_vals
    plt.plot(x_vals, y_vals, '--')

start_index = get_index(0.0)

end_index = get_index(10.0)


Cl_alpha, b1 = get_aerdynamic_coefficient(np.array(alpha_values[start_index:end_index]), np.array(Cl_values[start_index:end_index]))
Cd_alpha, b2 = get_aerdynamic_coefficient(np.array(alpha_values[start_index:end_index]), np.array(Cd_values[start_index:end_index]))
Cm_alpha, b3 = get_aerdynamic_coefficient(np.array(alpha_values[start_index:end_index]), np.array(Cm_values[start_index:end_index]))

print Cl_alpha, Cd_alpha, Cm_alpha

plt.figure()
plt.plot(alpha_values, Cl_values)
plt.show()

plt.figure()
plt.plot(alpha_values, Cm_values)
plt.show()

plt.figure()
plt.plot(alpha_values, Cd_values)
plt.show()
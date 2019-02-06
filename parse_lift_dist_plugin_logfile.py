'''To parse logfiles created from the lift distribution plugin'''


import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


N_iterations = 10
N_segments = 4

all_updates = np.empty((N_iterations,N_segments+1,1))
circulation_lines = np.empty((N_iterations,N_segments+1))


# circulation_values_arr = np.empty((N_iterations,N_segments+1))
CIRCULATION_VALUES_NEXT = False
START_NEW_UPDATE = False

df = pd.read_fwf("output.txt")
update_iterator = 0

reproduced_update_results = np.empty((N_iterations,N_segments+1,1))




with open("output.txt","r") as f:
    for line in f:
        if line[0:7] == "Forward":
            forward = line[9:15]
        if line[0:6] == "Upward":
            upward = line[8:14]
        if line[0:4] == "Span":
            span = float(line[6:10])

        if line[0:6] == "Gamma0":
            gamma0 = float(line[8:12])

        if line[0:10] == "NEW UPDATE":

            circulation_values_arr = np.empty((N_iterations, N_segments + 1))
            shape_x = np.shape(circulation_values_arr)[0]
            shape_y = np.shape(circulation_values_arr)[1]
            circulation_values_arr = np.reshape(circulation_values_arr, (shape_x, shape_y, 1))

            # print "The shape of the all_updates thing: ", np.shape(all_updates)
            # print "The shape of circulation_values_arr ", np.shape(circulation_values_arr)

            all_updates = np.dstack((all_updates,circulation_values_arr))
            update_iterator = update_iterator + 1


            # print update_iterator

        if line[0] == "~":
            START_LIFT_VALUES = True

        if line[0:3] == "***":
            CIRCULATION_VALUES_NEXT = True
            continue
        if CIRCULATION_VALUES_NEXT:
            circulation_values = line.strip().split(",")

            if circulation_values[-1] == '':
                circulation_values = [float(value) for value in circulation_values[:-1]]
            else:
                circulation_values = [float(value) for value in circulation_values]

            circulation_values_arr = np.array(circulation_values)
            circulation_values_arr = np.reshape(circulation_values_arr,(1,N_segments+1))

            np.vstack((circulation_lines, circulation_values_arr))

            CIRCULATION_VALUES_NEXT = False




    ''' Reproduce the results '''

print upward
print forward
print span






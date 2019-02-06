'''This python script is to parse the logfile created from the verify lift drag plugin'''

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from math import *
from parse_airfoil_data_file import parse_airfoil_data

lift_values = []
drag_values = []
aoa_values = []
cl_retrieved_lookup_list = []

with open("verify_liftdrag.txt","r") as f:
    for line in f:
        if line[0:3] == "Cl:":
            cl_retrieved_lookup = float(line[4:])
        if line[0:3] == "Rho":
            rho = float(line[5:])
        if line[0:4] == "Area":
            area = float(line[6:])
        if line[0:11] == "Groundspeed":
            V_groundspeed = line[13:]
            V_groundspeed = V_groundspeed.strip().split(" ")

            V_groundspeed = [float(item) for item in V_groundspeed]
            print "HERE ", V_groundspeed
        if line[0:4] == "Test": # e.g. Test: 4
            test_number = int(line[6:])
            print test_number
        if line[0:9] == "Magnitude": # e.g. Magnitude: 10.000000
            magnitude = float(line[11:])
            print magnitude
        if line[0:7] == "Azimuth": # e.g. Azimuth: 0.000000
            azimuth = float(line[9:])
            print azimuth
        if line[0:9] == "Elevation": # e.g. Elevation: -0.180000
            elevation = float(line[11:])
            print elevation

        if line[0:7] == "Force x": # e.g. Force x: 4.78485
            force_x = float(line[9:])
            print force_x
        if line[0:7] == "Force y":
            force_y = float(line[9:])
            print force_y
        if line[0:7] == "Force z":
            force_z = float(line[9:])
            print force_z
            ### Get the AoA after having obtained everything else ###
            aoa = elevation * np.cos(azimuth)
            print "AoA is ", aoa
            print "Cosine of aoa is: ", np.cos(aoa)

            ## Obtain the lift and drag based on the forceX and forceY and knowing the AoA: L*sin(alpha) - D*cos(alpha) = Fx L*cos(alpha) + D*sin(alpha) = Fy
            force_x_adjusted = -1*force_x
            force_z_adjusted = -1*force_z #+ 9.8*1.4


            D_calculated = force_z_adjusted * np.sin(aoa) - force_x_adjusted * np.cos(aoa)
            L_calculated = force_x_adjusted * np.sin(aoa) + force_z_adjusted * np.cos(aoa)


            lift_values.append(L_calculated)
            drag_values.append(D_calculated)
            aoa_values.append(aoa)
            cl_retrieved_lookup_list.append(cl_retrieved_lookup)
            print "The calculated drag is: ", D_calculated
            print "The calculated lift is: ", L_calculated

        if line[0:8] == "cosAlpha":
            cos_aoa_calculated_plugin = float(line[13:])
            print "COSINE of AoA calculated in the plugin: ", cos_aoa_calculated_plugin



## Create the plot from the XFOIL file ##
aoa_values_arr = np.array(aoa_values)
lift_values_arr = np.array(lift_values)
drag_values_arr = np.array(drag_values)
cl_retrieved_lookup_arr = np.array(cl_retrieved_lookup_list)
# Todo: change the hardcoded dimension
print "The length of the aoa array is ", np.size(aoa_values_arr)
print "The size of cl retrieved list is: ", np.size(cl_retrieved_lookup_arr)

print "look here", np.size(aoa_values_arr)

if np.size(aoa_values_arr)%17 == 0:

    aoa3 = aoa_values_arr
    cl3 = cl_retrieved_lookup_arr


    aoa_values_arr = np.reshape(aoa_values_arr,(np.floor(np.size(aoa_values_arr) / 17),17)) # reshaping so that the values can be averaged later
    lift_values_arr = np.reshape(lift_values_arr, (np.floor(np.size(aoa_values_arr) / 17),17))
    drag_values_arr = np.reshape(drag_values_arr, (np.floor(np.size(aoa_values_arr) / 17),17))
    cl_retrieved_lookup_arr = np.reshape(cl_retrieved_lookup_arr, (np.floor(np.size(aoa_values_arr) / 17),17))

    print "1 Lift values array", lift_values_arr
    # Take the average across the rows:
    aoa_values_print = np.mean(aoa_values_arr, axis=0)
    lift_values_print = np.mean(lift_values_arr, axis=0)
    drag_values_print = np.mean(drag_values_arr, axis=0)
    cl_retrieved_lookup_print = np.mean(cl_retrieved_lookup_arr, axis=0)





else:

    aoa3 = aoa_values_arr
    cl3 = cl_retrieved_lookup_arr

    upper_index = int(np.floor(np.size(aoa_values_arr)/17)*17) # upper index to get rid of last final values
    print "upper index ", upper_index
    aoa_values_arr_sub = np.array(aoa_values_arr[:upper_index])
    lift_values_arr_sub = np.array(lift_values[:upper_index])
    drag_values_arr_sub = np.array(drag_values[:upper_index])
    cl_retrieved_lookup_arr_sub = np.array(cl_retrieved_lookup_list[:upper_index])




    aoa_values_arr_sub = np.reshape(aoa_values_arr_sub,(int(np.floor(np.size(aoa_values_arr)/17)),17))
    lift_values_arr_sub = np.reshape(lift_values_arr_sub, (int(np.floor(np.size(aoa_values_arr) / 17)),17))
    drag_values_arr_sub = np.reshape(drag_values_arr_sub, (int(np.floor(np.size(aoa_values_arr) / 17)),17))
    cl_retrieved_lookup_arr_sub = np.reshape(cl_retrieved_lookup_arr_sub, (int(np.floor(np.size(aoa_values_arr) / 17)),17))

    print "2 Lift values array", lift_values_arr_sub

    # Take the average across the rows:
    aoa_values_print = np.mean(aoa_values_arr_sub, axis=0)
    lift_values_print = np.mean(lift_values_arr_sub, axis=0)
    drag_values_print = np.mean(drag_values_arr_sub, axis=0)
    cl_retrieved_lookup_print = np.mean(cl_retrieved_lookup_arr_sub, axis=0)





# Divide the lift values by the dynamic pressure 0.5*rho*V^2*area

coefficient = 0.5*rho * area * (magnitude)**2
alpha_values, Cl_values, Cd_values, Cm_values = parse_airfoil_data()

########################################################################################################################
################################# Log the values being plotted to a txt file ###########################################
########################################################################################################################





print (np.shape(aoa_values_arr))
print (np.shape(lift_values_arr/coefficient))
print (np.shape(drag_values_arr/coefficient))
print (np.shape(np.array(alpha_values)*np.pi/180.))
print (np.shape(Cl_values))
print (np.shape(Cd_values))
print (np.shape(aoa3))
print (np.shape(cl3))

d  = dict(alpha1 = aoa_values_arr, Cl1 = lift_values_arr/coefficient, Cd1 = drag_values_arr/coefficient, alpha2 = np.array(alpha_values)*np.pi/180., Cl2 = Cl_values, Cd2 = Cd_values, alpha3 = aoa3, Cl3 = cl3)
df = pd.DataFrame.from_dict(d, orient='index').transpose().fillna('')
df.to_csv('valuesToPlot2.csv', index=False, header=True)


plt.figure()

plt.scatter(aoa_values_arr, lift_values_arr/coefficient) #aoa_values_print, lift_values_print/coefficient
plt.scatter(aoa_values_arr, drag_values_arr/coefficient)
plt.scatter(np.array(alpha_values)*np.pi/180., Cl_values) #
plt.scatter(np.array(alpha_values)*np.pi/180., Cd_values)
plt.scatter(aoa_values_print, cl_retrieved_lookup_print)

plt.gca().legend(('Cl from lift','Cd from drag', 'Cl_values correct (GT)', 'Cd_values correct (GT)', 'cl_retrieved_lookup_print in plugin'))



plt.show()


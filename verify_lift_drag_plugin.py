import matplotlib.pyplot as plt
import numpy as np

'''The Inputs and Default Values '''
file_read = open('rigid_wing/model.sdf', 'r').read()

def get_sdf_param(tag, file_read = file_read):
    start = file_read.find(tag) + len(tag)
    tag_closing = tag[0] + '/' + tag[1:]
    end = file_read.find(tag_closing)
    return file_read[start:end]

# From the SDF
alpha0 = float(get_sdf_param('<a0>'))
cla = float(get_sdf_param('<cla>'))
cda = float(get_sdf_param('<cda>'))
cma = float(get_sdf_param('<cma>'))
claStall = float(get_sdf_param('<cla_stall>'))
alphaStall = float(get_sdf_param('<alpha_stall>'))
cmaStall = float(get_sdf_param('<cma_stall>'))
cp = float(get_sdf_param('<cp>'))
area = float(get_sdf_param('<area>'))
rho = float(get_sdf_param('<air_density>'))

forward_string =  get_sdf_param('<forward>').split()
forward = [[float(item)] for item in forward_string]
forward = np.array(forward)
upward_string = get_sdf_param('<upward>').split()
upward = [[float(item)] for item in upward_string]
upward = np.array(upward)

pose = [[0, 0, 1],[0, 0.75, 0]]

# From Commandline
vel_wind = 5
azimuth_wind = 0

# Misc
initial_vel_inertial = [0,0,0]


# Hardcoded inside the C++ file
minRatio = -1.0
maxRatio = 1.0
useConstantDragCoefficient = True
cdaStall = 1.0


'''The Functions'''

def get_trans_matrix(roll, pitch, yaw):
    Rx = np.matrix([[1,0,0],[0,np.cos(roll),-1*np.sin(roll)],[0, np.sin(roll), np.cos(roll)]])
    Ry = np.matrix([[np.cos(pitch),0,np.sin(pitch)],[0,1,0],[-1*np.sin(pitch), 0, np.cos(pitch)]])
    Rz = np.matrix([[np.cos(yaw), -1*np.sin(yaw), 0],[np.sin(yaw),np.cos(yaw), 0],[0,0,1]])
    Rzyx = Rz * Ry * Rx
    return Rzyx


def get_sweep(spanwiseI, velI, min_ratio = minRatio, max_ratio = maxRatio):

    # Clamp the sweep angle
    velI_arr = np.transpose(np.array(velI))
    spanwiseI_arr = np.array(spanwiseI)
    a = velI_arr.reshape((1,3))
    b = spanwiseI_arr.reshape((3, 1))
    dot_product = np.dot(a, b)
    sinSweepAngle = np.clip(dot_product, min_ratio, max_ratio)
    sweep = np.arcsin(sinSweepAngle)

    # Truncate sweep to within + 90/ -90 degrees
    if np.absolute(sweep) > 0.5 * np.pi:
        if sweep > 0:
            sweep = sweep - np.pi
        else:
            sweep = sweep + np.pi

    return sweep


# def get_cl(alpha,cl_alpha = cla, cl_alpha_stall = claStall, alpha_stall = alphaStall, alpha_zero = alpha0, sweep_angle = SWEEP_ANGLE_DEFAULT):
#
#     if alpha > alpha_stall:
#         cl = cl_alpha * alpha_stall + cl_alpha_stall * (alpha - alpha_stall) * np.cos(sweep_angle)
#     elif alpha < -1*alpha_stall:
#         cl = -1 * cl_alpha * alpha_stall + cl_alpha_stall * (alpha - alpha_stall) * np.cos(sweep_angle)
#     else:
#         cl = cl_alpha * alpha * np.cos(sweep_angle)
#
#     return cl

def get_wind(vel_wind, azimuth_wind):

    return wind_vector

'''Overview of the Variables'''
'''
vel: airspeed (speed of the body relative to the wind)
velI: ground speed/ the speed in the world coordinates. NOTE THAT IT ENDS UP BEING NORMALIZED.
forward: direction parallel to the chord of the airfoil
forwardI: points towards zero alpha, so is along the direction of the airspeed, and since the groundspeed is zero here, will be along the direction of the wind.
upward: perp to forward
upwardI: perp to forwardI
spanwiseI: is forwardI x upwardI (but why?)
velInLDPlane:  as the name suggests (but is the component of the AIRSPEED in the LDPlane)
AoA: angle between velI projected into lift-drag plane and forward vector.
So I: is along and perpendicular to the airspeed.
'''

'''The Actual WorkFlow'''

# Get the constant wind

wind = [5,0,0] # get_wind(vel_wind,azimuth_wind)

# Get vel (by adding the constant wind)

vel = initial_vel_inertial # obtained from this->link->WorldLinearVel(this->cp) What does WorldLinearVel do? Get the wind velocity at an entity location in the world coordinate frame. ignition::math::Vector3d WorldLinearVel
vel = np.array(vel) + np.array(wind) # but if vel is ground speed, why then is the velocity added to it and not subtracted from it?

# Get velI, the velocity of the body in interial ref frame, which is the initial velocity provided. Then normalizing it would give:

velI = initial_vel_inertial

if np.linalg.norm(np.array(velI)) != 0:
    velI = velI/np.linalg.norm(np.array(velI))
else:
    velI = velI
print "velI is: ", velI

# Get forwardI from forward, but what is forwardI? forwardI points toward zero alpha, so is along the direction of the wind
# forward: Normally, th/s is taken as a direction parallel to the chord of the airfoil in zero angle of attack forward flight.

euler_angle_list = pose[1][:]

forwardI = get_trans_matrix(*euler_angle_list) * forward

print "forwardI is: ", forwardI

# [Similarly] Get upwardI from upward

upwardI = get_trans_matrix(*euler_angle_list) * upward

print "upwardI is: ", upwardI

# Get the spanwiseI


spanwiseI = np.cross(np.transpose(np.array(forwardI)), np.transpose(np.array(upwardI)))

print "spanwiseI: ", spanwiseI

# Get the Sweep Angle

sweepAngle = get_sweep(spanwiseI, velI)
cosSweepAngle = np.cos(sweepAngle)

print "Sweep:", sweepAngle


# Get the velInLDPlane (remove the component of the velocity that is not in the LD Plane.
velInLDPlane = vel - np.transpose(np.dot(vel,np.transpose(spanwiseI)) * np.transpose(spanwiseI) /np.linalg.norm(np.array(spanwiseI))) # note that it is different from the C++ code here.

print "velInLDPlane is: ", velInLDPlane

# Get the drag direction

dragDirection = -1*velInLDPlane

print "dragDirection: ", dragDirection

# Get the direction of the lift

liftI = np.cross(spanwiseI,velInLDPlane)
if np.linalg.norm(liftI) != 0 :
    liftI = liftI/np.linalg.norm(liftI)
print "The direction of lift, liftI is: ", liftI

# The direction of the moment

momentDirection = spanwiseI


''' OT TUKA PO4VAME '''
# cosAlpha (line 300)

print "the dot product", np.dot(liftI, upwardI) # should be the dot product 0.731689
cosAlpha = np.clip(np.dot(liftI, upwardI), minRatio, maxRatio)
print "cosAlpha: ", cosAlpha

# Determine the sign of alpha, because cosAlpha does is ambiguous to that. If the liftI and forwardI are in the same direction,
#  then alpha is positive, since the I direction is perpendicular to the airspeed.


if np.dot(liftI, forwardI) > 0:
    alpha = alpha0 + np.arccos(cosAlpha)
else:
    alpha = alpha0 - np.arccos(cosAlpha)


# Normalize to within +90/-90 degrees. BUT WHY DO THAT?
while alpha > 0.5* np.pi:
    if alpha > 0:
        alpha = alpha - np.pi
    else:
        alpha = alpha + np.pi

print "Alpha is: ", alpha

# get the mangitude of the airspeed vector in the LD plane. THE ASSUMPTION LD PLANE IS PLANE OF SYMMETRY, WHAT ABOUT SIDEWAYS FORCES?
speedInLDPlane =  np.linalg.norm(velInLDPlane)

# compute the dynamic pressure
q =  0.5 * rho *  speedInLDPlane * speedInLDPlane

#  Calculate Cl
def get_cl(alpha,cla = cla, claStall = claStall, alphaStall = alphaStall,  cosSweepAngle = cosSweepAngle):
    if alpha > alphaStall:
        cl = cla * alphaStall + claStall * (alpha - alphaStall) * cosSweepAngle
        cl = np.maximum(0.0,cl) #WHY DO THIS?
    elif alpha < -alphaStall:
        cl = -1 * cla * alphaStall + claStall * (alpha + alphaStall) * cosSweepAngle

        print "cla", cla, "alphaStall", alphaStall, "claStall", claStall, "alpha", alpha,  "cosSweepAngle", cosSweepAngle
        cl = np.minimum(0.0, cl) #WHY DO THIS?
    else:
        cl = cla * alpha * cosSweepAngle

    print "Cl is: ", cl, "for alpha: ", alpha
    return cl

# Calculate Cd

def get_cd(alpha,cda = cda, cdaStall = cdaStall, alphaStall = alphaStall,  cosSweepAngle = cosSweepAngle):
    if alpha > alphaStall:
        if useConstantDragCoefficient is False:
            cd = cda * alphaStall + cdaStall * (alpha - alphaStall) * cosSweepAngle
        else:
            cd = (cda + cdaStall) * cosSweepAngle

    elif alpha < -alphaStall:
        if useConstantDragCoefficient is False:
            cd = -1 * cda * alphaStall + claStall * (alpha + alphaStall) * cosSweepAngle

            print "cda", cda, "alphaStall", alphaStall, "cdaStall", cdaStall, "alpha", alpha,  "cosSweepAngle", cosSweepAngle
        else:
            cd = (-1*cda + cdaStall) * cosSweepAngle
    else:
        if useConstantDragCoefficient is False:
            cd = cda * alpha * cosSweepAngle
        else:
            cd =  cda * cosSweepAngle # but if constant drag coefficient, shouldnt cda be zero?

    cd = np.abs(cd)

    print "Cd is: ", cd, "for alpha: ", alpha
    return cd

# Get the lift
cl = get_cl(alpha)
cd = get_cd(alpha)
lift = cl * q * area * liftI

print "lift is: ", lift

## Make the plot

list_of_alpha = np.arange(0, 2*alphaStall, 0.01)

# hl, = plt.plot([], [])
#
# def update_line(hl, new_data):
#     hl.set_xdata(np.append(hl.get_xdata(), new_data))
#     hl.set_ydata(np.append(hl.get_ydata(), new_data))
#     plt.draw()

alpha_values = []
cl_values = []
for angle in list_of_alpha:
    cl_iter = get_cl(angle)
    # new_data = [angle, cl_iter]
    # update_line(hl, new_data)

    cl_iter = cl_iter[0][0]
    alpha_values.append(angle)
    cl_values.append(cl_iter)

plt.figure()
plt.title("Plot of Cl vs Alpha")
plt.scatter(alpha_values, cl_values)
plt.axvline(x=alphaStall)
plt.show()


alpha_values = []
cd_values = []
for angle in list_of_alpha:
    cd_iter = get_cd(angle)
    # new_data = [angle, cl_iter]
    # update_line(hl, new_data)

    cd_iter = cd_iter[0][0]
    alpha_values.append(angle)
    cd_values.append(cd_iter)

plt.figure()
plt.title("Plot of Cd vs Alpha")
plt.scatter(alpha_values, cd_values)
plt.axvline(x=alphaStall)
plt.show()

print cd






























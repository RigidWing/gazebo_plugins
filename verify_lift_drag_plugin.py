import matplotlib.pyplot as plt
import numpy as np

'''The Inputs and Default Values '''

CL_ALPHA_DEFAULT = 0.0
CL_ALPHA_STALL_DEFAULT = 0.0
ALPHA_STALL_DEFAULT = 0.0
ALPHA_ZERO_DEFAULT = 0.0
SWEEP_ANGLE_DEFAULT = 0.0
MIN_RATIO_DEFAULT = -1.0
MAX_RATIO_DEFAULT = 1.0

pose = [[0, 0, 1],[0, 0.75, 0]]
vel_wind = 5
azimuth_wind = 0
initial_vel_inertial = [0,0,0]

'''The Functions'''

def toQuaternion(roll, pitch, yaw):

    # Abbreviations for the various angular functions
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)

    w = cy * cr * cp + sy * sr * sp
    x = cy * sr * cp - sy * cr * sp
    y = cy * cr * sp + sy * sr * cp
    z = sy * cr * cp - cy * sr * sp

    q = [w, x, y, z]
    return q



def get_trans_matrix(roll, pitch, yaw):
    Rx = np.matrix([[1,0,0],[0,np.cos(roll),-1*np.sin(roll)],[0, np.sin(roll), np.cos(roll)]])
    Ry = np.matrix([[np.cos(pitch),0,np.sin(pitch)],[0,1,0],[-1*np.sin(pitch), 0, np.cos(pitch)]])
    Rz = np.matrix([[np.cos(yaw), -1*np.sin(yaw), 0],[np.sin(yaw),np.cos(yaw), 0],[0,0,1]])
    Rzyx = Rz * Ry * Rx
    return Rzyx


def get_sweep(spanwiseI, velI, min_ratio = MIN_RATIO_DEFAULT, max_ratio = MAX_RATIO_DEFAULT):

    # Clamp the sweep angle
    velI_arr = np.array(velI)
    velI_arr_reshape = velI_arr.reshape((3,1))
    spanwiseI_arr = np.transpose(np.array(spanwiseI))
    dot_product = np.dot(spanwiseI_arr,velI_arr_reshape)
    sinSweepAngle = np.clip(dot_product, min_ratio, max_ratio)
    sweep = np.arcsin(sinSweepAngle)

    # Truncate sweep to within + 90/ -90 degrees
    if np.absolute(sweep) > 0.5 * np.pi:
        if sweep > 0:
            sweep = sweep - np.pi
        else:
            sweep = sweep + np.pi

    return sweep


def get_cl(alpha,cl_alpha = CL_ALPHA_DEFAULT, cl_alpha_stall = CL_ALPHA_STALL_DEFAULT, alpha_stall = ALPHA_STALL_DEFAULT, alpha_zero = ALPHA_ZERO_DEFAULT, sweep_angle = SWEEP_ANGLE_DEFAULT):

    if alpha > alpha_stall:
        cl = cl_alpha * alpha_stall + cl_alpha_stall * (alpha - alpha_stall) * np.cos(sweep_angle)
    elif alpha < -1*alpha_stall:
        cl = -1 * cl_alpha * alpha_stall + cl_alpha_stall * (alpha - alpha_stall) * np.cos(sweep_angle)
    else:
        cl = cl_alpha * alpha * np.cos(sweep_angle)

    return cl

def get_wind(vel_wind, azimuth_wind):

    return wind_vector



'''The Actual WorkFlow'''

# Get the constant wind

wind = [5,0,0] # get_wind(vel_wind,azimuth_wind)

# Get vel (by adding the constant wind)

vel = initial_vel_inertial #= this->link->WorldLinearVel(this->cp) What does WorldLinearVel do? Get the wind velocity at an entity location in the world coordinate frame. ignition::math::Vector3d WorldLinearVel
vel = vel + wind

# Get velI, the velocity of the body in interial ref frame, which is the initial velocity provided. Then normalizing it would give:

velI = initial_vel_inertial

if np.linalg.norm(np.array(velI)) != 0:
    velI = velI/np.linalg.norm(np.array(velI))
else:
    velI = velI
print "velI is: ", velI
# Get forwardI from forward


euler_angle_list = pose[1][:]

forwardI = get_trans_matrix(*euler_angle_list) * np.array([[1],[0],[0]])

print "forwardI is: ", forwardI

# [Similarly] Get upwardI from upward

upwardI = get_trans_matrix(*euler_angle_list) * np.array([[0],[0],[1]])

print "upwardI is: ", upwardI

# Get the spanwiseI

spanwiseI = np.cross(np.array(forwardI), np.array(upwardI), axis = 0)

print "spanwiseI: ", spanwiseI

# Get the Sweep Angle

sweepAngle = get_sweep(spanwiseI, velI)

print "Sweep:", sweepAngle


# Get the velInLDPlane (remove the component of the velocity that is not in the LD Plane.

velInLDPlane = vel - np.dot(vel,spanwiseI) * velI







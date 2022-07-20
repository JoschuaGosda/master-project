import numpy as np
import copy
from enum import Enum

def get_step():
    p1 = np.zeros((800, 3))
    v1 = np.zeros((800, 3))
    p2 = np.zeros((800, 3))
    v2 = np.zeros((800, 3))
    phi_delta = np.zeros((800, 3))
    dphi = np.zeros((800, 3))

    p1[50:800, 0] = 0.01
    v1[49:50, 0]  = 0.01/(1.0/80.0) # equals 0.8 m/s

    p2[50:800, 0] = 0.01
    p2[:, 2] = 0.4 # corresponds to -0.4 in y axis after the placement to yumi workspace
    v2[49:50, 0]  = 0.01/(1.0/80.0) # equals 0.8 m/s

    return p1, v1, p2, v2, phi_delta, dphi

def get_trajectory():
    try:
        data = np.load('data/traj_data.npy')
    except FileNotFoundError:
        print('It seems like you need to run the script first that generate the requested data')

    p1 = data[:, 0:3]
    v1 = data[:, 3:6]
    p2 = data[:, 6:9]
    v2 = data[:, 9:12]
    phi_delta = data[:, 12:15]
    dphi = data[:, 15:18]
    return p1, v1, p2, v2, phi_delta, dphi

def get_desJoints_L():
    try:
        comp_conf_left = np.load('data/compJointAngles_left.npy')
    except:
        print('It seems like you need to run the script first that generate the requested data')
    return comp_conf_left

def get_desJoints_R():
    try:
        comp_conf_right = np.load('data/compJointAngles_right.npy')
    except:
        print('It seems like you need to run the script first that generate the requested data')
    return comp_conf_right

def transform2yumi_workspace(p1, v1, p2, v2, phi_delta, dphi):
    for m in [p1, v1, p2, v2, phi_delta, dphi]:
        copy_col = copy.copy(m[:, 2]) # copy z
        m[:, 2] = m[:, 1] # shift y to z
        m[:, 1] = -copy_col # copy z to y
    return p1, v1, p2, v2, phi_delta, dphi

def place_trajectory(start_des, p1, p2):
    p_start = p1[0, :]
    offset = start_des - p_start

    # apply offset to all position coordinates
    for i in range(len(p1[:,0])):
        p1[i,:] = p1[i,:] + offset
        p2[i,:] = p2[i,:] + offset
    
    return p1, p2

# rearrange the logic joint values to the strange convention abb has
def logic2abb(joint_values):
    return joint_values[[0, 1, 3, 4, 5, 6, 2]]

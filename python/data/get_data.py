import numpy as np

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
        comp_conf_left = np.load('data/desJointAngles_left.npy')
    except:
        print('It seems like you need to run the script first that generate the requested data')
    return comp_conf_left

def get_desJoints_R():
    try:
        comp_conf_right = np.load('data/desJointAngles_right.npy')
    except:
        print('It seems like you need to run the script first that generate the requested data')
    return comp_conf_right
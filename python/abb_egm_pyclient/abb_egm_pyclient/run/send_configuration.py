#!/usr/bin/env python
import sys
from typing import Sequence
import time

import numpy as np
import numpy.typing as npt

from abb_egm_pyclient import EGMClient

DEFAULT_UDP_PORT = 6510


def send_configuration(port: int, target_conf: Sequence[float], joint_vel=1.0) -> None:
    """Move robot to target configuration
    takes sequence of target joints, compares them to current joint values and creates several movement
    messages by doing a linear decomposition of delta angles based on message frequency and allowed joint velocity

    Parameters
    ----------
    target_configuration
        List of joint positions in degrees
    joint_vel
        Max joint velocity in degrees/s for all joints
    """
    # send rate in hz
    # TODO: rate seems to change velocity in robotStudio - why?
    rate = 10

    egm_client = EGMClient(port=port)

    # set start_conf to the current values of joints
    pb_robot_msg = egm_client.receive_msg()
    start_conf: Sequence[float] = pb_robot_msg.feedBack.joints.joints

    if len(start_conf) == len(target_conf) - 1:
        start_conf.append(pb_robot_msg.feedBack.externalJoints.joints[0])

    start_conf_arr = np.array(start_conf)
    target_conf_arr = np.array(target_conf)

    # expression is cast to np.ArrayLike[.]
    deltas: np.ArrayLike[float] = target_conf_arr - start_conf_arr

    # get the highest delta, all the rest is dependant on that
    max_delta: float = np.fabs(deltas).max()

    # create several messages guiding to the final joint positions
    num_msgs = round(max_delta / joint_vel * rate)

    for n in range(num_msgs):
        sTime = time.time()
        pb_robot_msg = egm_client.receive_msg()
        cur_configuration: Sequence[float] = pb_robot_msg.feedBack.joints.joints
        # add the 7th joint to the configuration
        cur_configuration.append(pb_robot_msg.feedBack.externalJoints.joints[0])

        print(f"Current configuration {cur_configuration}")

        conf = []
        # parallel iteration, linear decomposition of each joints delta
        for start_pos, delta in zip(start_conf, deltas):
            abs_change = n * delta / num_msgs
            if delta < 0:
                conf.append(start_pos - abs_change)
            else:
                conf.append(start_pos + abs_change)

        egm_client.send_planned_configuration(conf)

        # take the execution time into account that loops stays iterating with the rate frequency
        # get more important the higher rate becomes like 100-250
        sleeping_time = 1/rate - (time.time()-sTime)
        time.sleep(sleeping_time) 

    # after all messages are sent out, wait for 10 sec and check if positions converged
    n = 0
    while n < 10:
        pb_robot_msg = egm_client.receive_msg()
        if pb_robot_msg.mciConvergenceMet:
            print("Joint positions converged.")
            break
        else:
            print("Waiting for convergence.")
        n += 1
        time.sleep(1)
    else:
        raise TimeoutError(f"Joint positions did not converge after {n} s timeout.")


if __name__ == "__main__":
    target_configuration = None

    args = sys.argv[1:]  # first argument is just path to file

    if len(args) == 0:
        target_configuration = [30.0, 30.0, 40.0, 30.0, 10.0, 15.0]
    elif len(args) in (6, 7):  # check for six to seven joint values
        target_configuration = [float(n) for n in args]
    else:
        raise RuntimeError("Wrong number of arguments, need six or seven joint values")

    send_configuration(target_configuration)

#!/usr/bin/env python
import sys
from typing import Sequence
import time

import numpy as np

from abb_egm_pyclient import EGMClient

'''
The example of the repo https://gitlab.control.lth.se/tetov/abb_egm_pyclient is a good starting point but
code needs to be adapted to projects goal of foam cutting. Structure of functions needs to changed since
further steps like computing the IK needs to be performed between receiving and sending positions and 
reading force sensor data.

'''

DEFAULT_UDP_PORT = 6510


# TODO: read the data from g-code and transform it into the global frame of yumi
# make sure to set the first configuration as the fine point in robot studio, so that they
# virtual and real configuration are synchronized

# TODO: loop over the g-code positions (check g-code of its based on constant speed)
# instructions in G code are G1 instruction meaning moving at a constant feed (velocity in axis direction)
# https://www.seniorcare2share.com/how-does-g-code-work/

    # TODO: read current configuration via egm client

    # TODO: use RL to compute the current jacobian

    # TODO: call the C++ function to get the joint values
    #   input:  current jacobian, nullspace gradient, desired position in task space, sample time 

    # TODO: send to obtained joint values via egm client
import socket
import logging
from typing import Any, Tuple

from abb_egm_pyclient.atomic_counter import AtomicCounter

from abb_egm_pyclient.egm_pb2 import EgmRobot, EgmSensor, EgmHeader

import numpy as np
import numpy.typing as npt

log = logging.getLogger(__name__)


class EGMClientException(Exception):
    """Base class for other exceptions"""

    pass


class EGMClientNotInitializedException(EGMClientException):
    """When user tries to access attributes set in self.recieve_from_robot()"""

    pass


class EGMClient:
    """Communication client to ABB EGM interface.

    Parameter
    ---------
    port
        Port number, same as defined in Controller > Configuration
        > Communication > Transmission Protocol > {name of your UDP configuration}

    Attributes
    ----------
    socket
        UDP socket used for communication
    robot_controller_address
        IP address to controller, comes from first packet recieved
    send_counter
        An atomic counter used for sequence numbers for outbound packages
    """

    def __init__(self, port=6510):
        self.socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        self.robot_controller_address = None
        self.send_counter = AtomicCounter()

        self.socket.bind(("", port))

    def _get_last_packet(self) -> Tuple[bytes, Any]:
        last_recieved = (None, None)

        self.socket.setblocking(False)

        while True:
            try:
                data, addr = self.socket.recvfrom(2048)  # read from socket
                if data:
                    last_recieved = (data, addr)  # store last data and addr
            except socket.error as err:
                # EWOULDBLOCK: recvfrom would block since socket buffer is empty
                if err.args[0] == socket.EWOULDBLOCK:
                    if last_recieved[0]:  # if last data got picked up
                        break
                    else:
                        continue  # else wait for data
                else:
                    self.socket.setblocking(True)
                    raise err  # if there is another error, raise exception

        self.socket.setblocking(True)
        return last_recieved

    def receive_msg(self) -> EgmRobot:
        """Get latest UDP packet from IRC5."""

        data, addr = self._get_last_packet()

        # the address used for sending to the robot is picked up here
        if not self.robot_controller_address or self.robot_controller_address != addr:
            self.robot_controller_address = addr

        # use the message description from the proto file to create a python
        # class from incoming data
        msg = EgmRobot()
        msg.ParseFromString(data)

        return msg

    def send_msg(self, msg: Any) -> None:
        """Send a protobuf message to the robot controller."""
        if not self.robot_controller_address:
            raise EGMClientNotInitializedException(
                "You need to start communication with controller."
            )

        self.socket.sendto(msg.SerializeToString(), self.robot_controller_address)

    def _create_sensor_msg(self, type="MSGTYPE_CORRECTION") -> EgmSensor:
        """Create EgmSensor message with sequence number

        type
            EgmSensor message type
        """
        msg = EgmSensor()
        msg.header.mtype = EgmHeader.MessageType.Value(type)
        msg.header.seqno = self.send_counter.inc()

        return msg

    def send_planned_configuration(self, configuration) -> None:
        """Send target configuration to robot controller.

        configuration
            List of joint position (angles) in degrees or radians, depending on
            RobotWare version.
        """
        msg = self._create_sensor_msg()

        conf_as_list = np.asarray(configuration).tolist()
        
        #TODO: why not just do the following in one line?
        # joints refer to the common six joints
        joints = conf_as_list[:6]
        # external joints are e.g. the 7th for the yumi
        external_joints = conf_as_list[6:]

        msg.planned.joints.joints.extend(joints)

        if len(external_joints) > 0:
            msg.planned.externalJoints.joints.extend(external_joints)



        self.send_msg(msg)

    def send_planned_frame(
        self, x: float, y: float, z: float, rx: float, ry: float, rz: float
    ) -> None:
        """Send target frame to robot controller.

        x
        y
        z
            Cartesian coordinates in mm(?)
        rx
        ry
        rz
            Euler angles in (?)
        """
        msg = self._create_sensor_msg()

        msg.planned.cartesian.pos.x = x
        msg.planned.cartesian.pos.y = y
        msg.planned.cartesian.pos.z = z
        msg.planned.cartesian.euler.x = rx
        msg.planned.cartesian.euler.y = ry
        msg.planned.cartesian.euler.z = rz

        self.send_msg(msg)

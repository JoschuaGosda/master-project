#!/usr/bin/env python
import logging

try:
    from abb_egm_pyclient import EGMClient
except ImportError:
    raise ImportWarning("abb_egm not found, have you installed the package?")

DEFAULT_UDP_PORT = 6510

log = logging.getLogger("egm_client")


def print_egm_feedback(port: int) -> None:
    """Print EGM feedback.

    Parameters
    ----------
    rate
        Frequency of prints in hertz.
    """

    egm_client = EGMClient(port=port)

    while True:
        try:
            msg = egm_client.receive_msg()
            print(f"Sequence number: {msg.header.seqno}")
            print(f"Body: {msg.feedBack}")

        except Exception as exc:
            log.error(f"Exception raised {exc}")
            log.info("Retrying")


if __name__ == "__main__":
    print_egm_feedback(port=DEFAULT_UDP_PORT)

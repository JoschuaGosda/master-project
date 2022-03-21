import argparse
from abb_egm_pyclient.run.print_egm_feedback import print_egm_feedback
from abb_egm_pyclient.run.send_configuration import send_configuration
from abb_egm_pyclient.run.send_pose import send_pose

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run one of the example EGM scripts.")
    parser.add_argument("port", type=int, help="UDP port")

    subparsers = parser.add_subparsers(help="sub-command help", dest="subparser_name")

    func_sel_mapping = {
        "print": print_egm_feedback,
        "joint": send_configuration,
        "pose": send_pose,
    }

    parser_print = subparsers.add_parser(
        "print",
        help="print messages recieved from the EGM interface on the controller",
    )

    parser_joint = subparsers.add_parser(
        "joint",
        help="print messages recieved from the EGM interface on the controller",
    )
    parser_joint.add_argument(
        "joint_values",
        nargs="*",
        type=float,
    )

    parser_pose = subparsers.add_parser(
        "pose",
        help="print messages recieved from the EGM interface on the controller",
    )
    parser_pose.add_argument(
        "pose_values", nargs=6, type=float, metavar=("x", "y", "z", "rx", "ry", "rz")
    )

    args = parser.parse_args()

    print(args.__dict__)

    if args.subparser_name == "print":
        print_egm_feedback(args.port)

    if args.subparser_name == "joint":
        if len(args.joint_values) not in (6, 7):
            raise RuntimeError("Incorrect number of joint values.")
        send_configuration(args.port, args.joint_values)

    if args.subparser_name == "pose":
        send_configuration(args.port, *args.pose_values)

    func = func_sel_mapping[args.func_selection]

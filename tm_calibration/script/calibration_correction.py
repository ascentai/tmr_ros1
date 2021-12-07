#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import yaml
import argparse
from tm_msgs.srv import AskItem
from tm_calibration.utils import urdf_DH_from_tm_DH, xyzrpys_from_urdf_DH


def run_calibration_correction(output_filename, service_uri):
    try:
        rospy.wait_for_service(service_uri, timeout=1)
        ask_item = rospy.ServiceProxy(service_uri, AskItem)
    except (rospy.ROSException, rospy.ServiceException):
        rospy.logerr(
            "Failed to connect to ROS driver service ({}). Ensure that the robot powered on, the ROS driver is running, and that the service URI is correct.".format(
                service_uri
            )
        )
        return

    res_dh = ask_item("dh", "DHTable", 1.0)
    res_dd = ask_item("dd", "DeltaDH", 1.0)

    dh_strs = res_dh.value[9:-1].split(",")
    dd_strs = res_dd.value[9:-1].split(",")

    if len(dh_strs) != 42:
        rospy.logerr(
            "Invalid DH. Received {} values but expected {}. Data is {}".format(
                len(dh_strs), 42, dh_strs
            )
        )
        return
    if len(dd_strs) != 30:
        rospy.logerr(
            "Invalid Delta DH. Received {} values but expected {}. Data is {}".format(
                len(dd_strs), 30, dd_strs
            )
        )
        return

    dh = [float(i) for i in dh_strs]
    dd = [float(i) for i in dd_strs]

    urdf_dh = urdf_DH_from_tm_DH(dh, dd)
    xyzs, rpys = xyzrpys_from_urdf_DH(urdf_dh)

    link_names = [
        "shoulder_1",
        "shoulder_2",
        "elbow",
        "wrist_1",
        "wrist_2",
        "wrist_3",
    ]

    out = {"kinematics": {}}
    for link_name, (xyz, rpy) in zip(link_names, zip(xyzs, rpys)):
        xyz, rpy = xyz.tolist(), rpy.tolist()
        out["kinematics"][link_name] = {
            "x": xyz[0],
            "y": xyz[1],
            "z": xyz[2],
            "roll": rpy[0],
            "pitch": rpy[1],
            "yaw": rpy[2],
        }

    with open(output_filename, "w") as f:
        yaml.safe_dump(out, f)

    rospy.loginfo("Successfully wrote calibration YAML to {}.".format(output_filename))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Extracts the factory calibration directly from a Techman Robot using the ROS driver, calculates the URDF correction and saves it into a .yaml file."
    )
    parser.add_argument(
        "--output_filename",
        type=str,
        required=True,
        help="Path to where the script should write the output.",
    )
    args = parser.parse_args()

    rospy.init_node("calibration_correction")

    run_calibration_correction(args.output_filename, "/tm_driver/ask_item")

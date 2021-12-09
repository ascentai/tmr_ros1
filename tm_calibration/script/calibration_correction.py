#! /usr/bin/env python
# -*- coding: utf-8 -*-

import re
import socket
import rospy
import yaml
import argparse
import time
from tm_calibration.utils import urdf_DH_from_tm_DH, xyzrpys_from_urdf_DH


def dh_values_from_robot(robot_ip, data):
    pattern = re.compile("{}=\{{(.*)\}}".format(data[1]))

    # The following were obtained by wiresharking the communication between the driver and the robot
    checksums = {"dd": "37", "dh": "3d"}
    payload = "$TMSVR,13,{},12,{},*{}\r\n".format(data[0], data[1], checksums[data[0]])

    robot_socket = socket.socket()
    robot_socket.settimeout(5)

    robot_socket.connect((robot_ip, 5891))
    robot_socket.sendall(payload)

    timeout = time.time() + 10
    received = robot_socket.recv(1024)
    match = pattern.search(received)

    while match is None:
        if time.time() > timeout:
            return []

        received = received + robot_socket.recv(1024)
        match = pattern.search(received)

    return [float(s) for s in match.group(1).split(",")]


def run_calibration_correction(output_filename, ip_address):
    try:
        dd = dh_values_from_robot(ip_address, ("dd", "DeltaDH"))
        dh = dh_values_from_robot(ip_address, ("dh", "DHTable"))
    except socket.error:
        rospy.logerr(
            "Failed to connect to robot. Ensure that the robot powered on and that IP adress({}) is correct.".format(
                ip_address
            )
        )
        return

    if len(dh) != 42:
        rospy.logerr(
            "Invalid DH. Received {} values but expected {}. Data is {}".format(len(dh), 42, dh)
        )
        return
    if len(dd) != 30:
        rospy.logerr(
            "Invalid Delta DH. Received {} values but expected {}. Data is {}".format(
                len(dd), 30, dd
            )
        )
        return

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

    parser.add_argument(
        "--robot_ip",
        type=str,
        required=True,
        help="Techman robot IP",
    )
    args = parser.parse_args()

    rospy.init_node("calibration_correction")

    run_calibration_correction(args.output_filename, args.robot_ip)

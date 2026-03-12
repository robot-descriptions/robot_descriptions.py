#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
#
# /// script
# dependencies = ["meshcat", "pin", "xacrodoc", "robot_descriptions"]
# ///

"""
Visualize an FR3 with a Robotiq 2F-85 attached at the flange.

This example renders the FR3 Xacro with `hand:=false`, merges in the standalone
Robotiq URDF, then adds a fixed joint from `fr3_link8` to
`robotiq_85_base_link`.
"""
import math
import os
import tempfile
import xml.etree.ElementTree as ET

import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer

from robot_descriptions import fr3_description, robotiq_2f85_description
from robot_descriptions._xacro import get_urdf_path

XYZ = (0.0, 0.0, 0.0)
RPY = (0.0, 0.0, math.pi / 4)


def _attach_gripper(
    fr3_root: ET.Element,
    gripper_root: ET.Element,
) -> ET.Element:
    for child in list(gripper_root):
        fr3_root.append(child)

    joint = ET.Element(
        "joint",
        attrib={
            "name": "fr3_robotiq_2f85_attachment",
            "type": "fixed",
        },
    )
    ET.SubElement(
        joint,
        "origin",
        attrib={
            "xyz": " ".join(str(value) for value in XYZ),
            "rpy": " ".join(str(value) for value in RPY),
        },
    )
    ET.SubElement(joint, "parent", attrib={"link": "fr3_link8"})
    ET.SubElement(joint, "child", attrib={"link": "robotiq_85_base_link"})
    fr3_root.append(joint)
    return fr3_root


def _visualize_with_meshcat(urdf_path: str) -> None:
    package_dirs = [
        os.path.dirname(fr3_description.REPOSITORY_PATH),
        os.path.dirname(robotiq_2f85_description.REPOSITORY_PATH),
    ]
    robot = pin.RobotWrapper.BuildFromURDF(
        filename=urdf_path,
        package_dirs=package_dirs,
    )
    robot.setVisualizer(MeshcatVisualizer())
    robot.initViewer(open=True)
    robot.loadViewerModel()
    robot.display(robot.q0)
    input("Press Enter to close MeshCat and terminate... ")


if __name__ == "__main__":
    fr3_root = ET.parse(
        get_urdf_path(
            fr3_description,
            xacro_args=fr3_description.XACRO_ARGS_NO_HAND,
        )
    ).getroot()
    gripper_root = ET.parse(robotiq_2f85_description.URDF_PATH).getroot()
    merged_root = _attach_gripper(fr3_root, gripper_root)

    temp_dir = tempfile.TemporaryDirectory(prefix="fr3_robotiq_2f85_")
    output_path = os.path.join(temp_dir.name, "fr3_robotiq_2f85.urdf")
    tree = ET.ElementTree(merged_root)
    ET.indent(tree, space="  ")
    tree.write(output_path, encoding="utf-8", xml_declaration=True)

    print(output_path)
    _visualize_with_meshcat(output_path)
    temp_dir.cleanup()

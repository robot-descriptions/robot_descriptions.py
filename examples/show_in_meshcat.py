#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

"""
Show a robot description selected from the command line using MeshCat.

This example is equivalent to `python -m robot_descriptions show_in_meshcat`.
It requires Pinocchio, installed by e.g. `conda install pinocchio`, and
MeshCat, installed by e.g. `conda install meshcat-python`.
"""

import argparse

try:
    from pinocchio.visualize import MeshcatVisualizer
except ImportError as e:
    raise ImportError("Pinocchio not found, try `pip install pin`") from e

from robot_descriptions.loaders.pinocchio import load_robot_description

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("name", help="name of the robot description")
    args = parser.parse_args()

    try:
        robot = load_robot_description(args.name)
    except ModuleNotFoundError:
        robot = load_robot_description(f"{args.name}_description")

    robot.setVisualizer(MeshcatVisualizer())
    robot.initViewer(open=True)
    robot.loadViewerModel()
    robot.display(robot.q0)

    input("Press Enter to close MeshCat and terminate... ")

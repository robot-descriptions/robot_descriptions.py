#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

"""
Show a robot description, specified from the command line, using PyBullet.

This example is equivalent to `python -m robot_descriptions show_in_pybullet`.
It requires PyBullet, which can be installed by `pip install pybullet`.
"""

import argparse

import pybullet

from robot_descriptions.loaders.pybullet import load_robot_description

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("name", help="name of the robot description")
    args = parser.parse_args()

    pybullet.connect(pybullet.GUI)
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_SHADOWS, 0)
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0)

    try:
        robot = load_robot_description(args.name)
    except ModuleNotFoundError:
        robot = load_robot_description(f"{args.name}_description")

    input("Press Enter to close PyBullet and terminate... ")
    pybullet.disconnect()

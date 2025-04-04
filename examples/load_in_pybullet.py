#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

"""
Load a robot description selected from the command line in PyBullet.

This example requires PyBullet, which is installed by `pip install pybullet`.
"""

import argparse

import pybullet

from robot_descriptions.loaders.pybullet import load_robot_description

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("name", help="name of the robot description")
    args = parser.parse_args()

    pybullet.connect(pybullet.DIRECT)

    try:
        robot = load_robot_description(args.name)
    except ModuleNotFoundError:
        robot = load_robot_description(f"{args.name}_description")

    print(f"Robot successfully loaded with an ID of {robot}")
    pybullet.disconnect()

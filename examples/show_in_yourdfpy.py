#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

"""
Show a robot description selected from the command line in yourdfpy.

This example is equivalent to `python -m robot_descriptions show_in_yourdfpy`.
It requires `yourdfpy`, an optional dependency that can be installed by `pip
install yourdfpy`.
"""

import argparse

from robot_descriptions.loaders.yourdfpy import load_robot_description

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("name", help="name of the robot description")
    args = parser.parse_args()

    try:
        robot = load_robot_description(args.name)
    except ModuleNotFoundError:
        robot = load_robot_description(f"{args.name}_description")

    robot.show()

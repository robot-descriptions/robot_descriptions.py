#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

"""
Show a robot description, specified from the command line, using yourdfpy.

Note:
    See ``robot_descriptions/_command_line.py`` for a more advanced
    implementation, including the ability to set the robot configuration or
    show collision meshes.

This example requires `yourdfpy` which is an optional dependency. It can be
installed separately (``pip install yourdfpy``), or when robot descriptions are
installed with optional dependencies ``pip install robot_descriptions[opts]``.
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

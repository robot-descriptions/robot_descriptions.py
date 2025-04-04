#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

"""
Load a robot description selected from the command line in MuJoCo.

This example requires MuJoCo, which is installed by `pip install mujoco`.
"""

import argparse

from robot_descriptions.loaders.mujoco import load_robot_description

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("name", help="name of the robot description")
    args = parser.parse_args()

    try:
        model = load_robot_description(args.name)
    except ModuleNotFoundError:
        model = load_robot_description(f"{args.name}_mj_description")

    print(f"Robot successfully loaded with model={model}")

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

"""
Show a robot description selected from the command line using MuJoCo.

This example is equivalent to `python -m robot_descriptions show_in_mujoco`. It
requires MuJoCo, which is installed by `pip install mujoco`, and the MuJoCo
viewer installed by `pip install mujoco-python-viewer`.
"""

import argparse

import mujoco

try:
    import mujoco_viewer
except ImportError as e:
    raise ImportError(
        "MuJoCo viewer not found, " "try `pip install mujoco-python-viewer`"
    ) from e

from robot_descriptions.loaders.mujoco import load_robot_description

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("name", help="name of the robot description")
    args = parser.parse_args()

    try:
        model = load_robot_description(args.name)
    except ModuleNotFoundError:
        model = load_robot_description(f"{args.name}_mj_description")

    data = mujoco.MjData(model)
    viewer = mujoco_viewer.MujocoViewer(model, data)
    mujoco.mj_step(model, data)  # step at least once to load model in viewer
    while viewer.is_alive:
        viewer.render()
    viewer.close()

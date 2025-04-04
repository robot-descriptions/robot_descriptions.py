#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

"""
Load a robot description selected from the command line in RoboMeshCat.

This example uses RoboMeshCat: https://github.com/petrikvladimir/RoboMeshCat
"""

import argparse

import numpy as np
import robomeshcat

from robot_descriptions.loaders.robomeshcat import load_robot_description

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("name", help="name of the robot description")
    args = parser.parse_args()

    try:
        robot = load_robot_description(args.name)
    except ModuleNotFoundError:
        robot = load_robot_description(f"{args.name}_description")

    scene = robomeshcat.Scene()
    scene.add_robot(robot)

    with scene.animation(fps=1):
        scene.render()
        robot[0] = -0.5 * np.pi
        scene.render()

        for t in np.linspace(0, 2 * np.pi, 10):
            scene.camera_pos = np.array([np.sin(t), np.cos(t), 1.0])
            scene.render()

        scene.reset_camera()  # back to user interaction
        scene.render()

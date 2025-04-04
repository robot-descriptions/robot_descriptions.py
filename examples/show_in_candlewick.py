#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2025 Inria

"""
Show a robot description selected from the command line in Candlewick.

This example requires Pinocchio, installed by e.g. `conda install pinocchio`,
and Candlewick: https://github.com/Simple-Robotics/candlewick.
"""

import argparse

try:
    from candlewick.multibody import Visualizer, VisualizerConfig
except ImportError as import_error:
    raise ImportError(
        "Candlewick not found, "
        "see https://github.com/Simple-Robotics/candlewick"
    ) from import_error

from robot_descriptions.loaders.pinocchio import load_robot_description

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("name", help="name of the robot description")
    parser.add_argument("--width", help="window width in px", default=1600)
    parser.add_argument("--height", help="window height in px", default=900)
    args = parser.parse_args()

    try:
        robot = load_robot_description(args.name)
    except ModuleNotFoundError:
        robot = load_robot_description(f"{args.name}_description")

    config = VisualizerConfig()
    config.width = args.width
    config.height = args.height
    visualizer = Visualizer(config, robot.model, robot.visual_model)
    while not visualizer.shouldExit:
        visualizer.display(robot.q0)

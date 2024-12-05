#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

"""
Load a robot description in a PyBullet physics simulation.

This example requires PyBullet, which can be installed with conda or pip.
"""

import argparse
import time

import pybullet
import pybullet_data
from robot_descriptions.loaders.pybullet import load_robot_description

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("name", help="name of the robot description")
    parser.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="duration of the simulation in seconds",
    )
    parser.add_argument(
        "--timestep",
        type=float,
        default=0.01,
        help="duration of a simulation timestep in seconds",
    )
    args = parser.parse_args()

    pybullet.connect(pybullet.GUI)
    pybullet.setGravity(0.0, 0.0, -9.81)
    pybullet.setTimeStep(args.timestep)
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 0)
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_SHADOWS, 0)
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0)
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
    plane_id = pybullet.loadURDF("plane.urdf")

    try:
        robot = load_robot_description(args.name)
    except ModuleNotFoundError:
        robot = load_robot_description(f"{args.name}_description")

    initial_position = [0.0, 0.0, 1.0]
    initial_orientation = [0.0, 0.0, 0.0, 1.0]
    pybullet.resetBasePositionAndOrientation(
        robot, initial_position, initial_orientation
    )
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 1)

    for _ in range(int(args.duration / args.timestep)):
        pybullet.stepSimulation()
        time.sleep(args.timestep)

    pybullet.disconnect()

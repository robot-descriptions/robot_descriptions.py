#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2025 Inria

"""
Show all quadruped robot descriptions using Pinocchio and MeshCat.
"""

try:
    import pinocchio as pin
    from pinocchio.visualize import MeshcatVisualizer
except ImportError as e:
    raise ImportError("Pinocchio not found, try `pip install pin`") from e

from robot_descriptions import DESCRIPTIONS
from robot_descriptions.loaders.pinocchio import load_robot_description

GRID_SPACING = 2.0  # [m]
GRID_SIZE = 5

if __name__ == "__main__":
    nb_quadrupeds = 0
    viewer = None
    visualizer = MeshcatVisualizer()

    for name, description in DESCRIPTIONS.items():
        if "quadruped" not in description.tags:
            continue
        if name == "a1_mj_description":
            # See https://github.com/stack-of-tasks/pinocchio/issues/2613
            continue
        if name in ("anymal_b_mj_description", "anymal_c_mj_description"):
            # See https://github.com/stack-of-tasks/pinocchio/issues/2611
            continue
        robot = load_robot_description(
            name,
            root_joint=pin.JointModelFreeFlyer(),
        )
        robot.setVisualizer(visualizer)
        if viewer is None:
            robot.initViewer(open=True)
            viewer = robot.viewer
        robot.loadViewerModel()
        row = nb_quadrupeds // GRID_SIZE
        column = nb_quadrupeds % GRID_SIZE
        nb_quadrupeds += 1
        q = robot.q0.copy()
        q[0] = row * GRID_SPACING  # [m]
        q[1] = column * GRID_SPACING  # [m]
        robot.display(q)

    print(f"Displaying {nb_quadrupeds} quadruped robot descriptions")
    input("Press Enter to close MeshCat and terminate... ")

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Show a robot descriptions specified from the command line, using MeshCat.

This example requires Pinocchio, which is installed by ``pip install pin``, and
MeshCat, which is installed by ``pip install meshcat``.
"""

import argparse

try:
    import pinocchio as pin
except ImportError as e:
    raise ImportError("Pinocchio not found, try ``pip install pin``") from e

from robot_descriptions.loaders.pinocchio import load_robot_description

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("name", help="name of the robot description")
    args = parser.parse_args()

    if args.name == "cf2_description":
        raise ValueError(
            "See https://github.com/stack-of-tasks/pinocchio/issues/1741"
        )

    try:
        robot = load_robot_description(args.name)
    except ModuleNotFoundError:
        robot = load_robot_description(f"{args.name}_description")

    configuration = pin.neutral(robot.model)
    viz = pin.visualize.MeshcatVisualizer(
        robot.model, robot.collision_model, robot.visual_model
    )
    robot.setVisualizer(viz, init=False)
    viz.initViewer(open=True)
    viz.loadViewerModel()
    viz.display(configuration)

    input("Press Enter to close MeshCat and terminate... ")
